"""Capture camera bytes in a separate process; generate datasets after stop.

The handoff thread sends original camera JPEGs and matching scalar samples.
The child acknowledges after writing the capture journal, without video encoding.
"""

from __future__ import annotations

import logging
import math
import multiprocessing as mp
import os
import signal
import threading
import time

logger = logging.getLogger(__name__)


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"[:2048]


def _recording_worker(config, observation_features, action_features, pipe):
    """Spawn target: own every LeRobot object and finish each episode here."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", force=True,
    )
    try:
        affinity = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    except OSError:
        affinity = None
    logger.info("Recording worker entered: PID=%d CPU affinity=%s", os.getpid(), affinity)
    sink = None
    try:
        while True:
            try:
                request, operation, payload = pipe.recv()
            except (EOFError, OSError):
                break
            shutting_down = operation == "shutdown"
            try:
                if operation == "open":
                    if sink is not None:
                        raise RuntimeError("Finish the current recording episode before opening another")
                    from .deferred_recording import DeferredEpisodeSink

                    sink = DeferredEpisodeSink(config, observation_features, action_features)
                elif operation == "add":
                    if sink is None:
                        raise RuntimeError("No recording episode is open")
                    observation, action, timestamp = payload
                    sink.add(observation, action, timestamp)
                    # Drop IPC image copies before ACK permits the next group.
                    del observation, action, payload
                elif operation in ("finish", "shutdown"):
                    if sink is not None:
                        try:
                            # Bind the loop variable so progress reports the request
                            # that owns this finish, even if it is called later.
                            def progress(stage, frames=0, total=None, request=request):
                                pipe.send((request, "progress", (stage, frames, total)))

                            sink.finish(progress=progress)
                        finally:
                            sink = None
                else:
                    raise ValueError(f"Unknown recording operation: {operation}")
            except Exception as exc:
                response = (request, False, _error_text(exc))
                logger.exception("Recording process operation %s failed", operation)
            else:
                response = (request, True, None)
            try:
                pipe.send(response)
            except (BrokenPipeError, EOFError, OSError):
                break
            if shutting_down:
                break
    except BaseException:
        logger.exception("Recording process failed")
        raise
    finally:
        if sink is not None:
            try:
                sink.finish()
            except Exception:
                logger.exception("Recording process could not finalize after parent disconnect")
        pipe.close()


class RemoteEpisodeSink:
    """Episode-scoped handle, used only by the parent recording worker."""

    def __init__(self, owner: RecordingProcess, episode: int):
        self._owner = owner
        self._episode = episode
        self._finished = False

    def add(self, observation: dict, action: dict, timestamp: float) -> None:
        if self._finished:
            raise RuntimeError("Recording episode is already finished")
        self._owner._add(self._episode, observation, action, timestamp)

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._owner._finish(self._episode)


class RecordingProcess:
    """Persistent disk capture process with one in-flight sample group.

    ``start`` only launches the child; ``open_episode`` prepares a capture journal.
    The caller must run episode operations in its recording worker, and call
    ``close`` after arm control has shut down. There is no shared RGB slot.
    """

    def __init__(
        self, config, observation_features: dict, action_features: dict,
        cpu_affinity: tuple[int, ...] | None = None,
    ):
        self.config = config
        self.observation_features = dict(observation_features)
        self.action_features = dict(action_features)
        self._timeout = config.finalize_timeout_s
        if not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError("Recording process timeout must be finite and positive")
        self._cpu_affinity = tuple(cpu_affinity) if cpu_affinity else None
        if self._cpu_affinity is not None and any(
            isinstance(cpu, bool) or not isinstance(cpu, int) or cpu < 0
            for cpu in self._cpu_affinity
        ):
            raise ValueError("Recording CPU affinity must contain nonnegative CPU integers")
        self._shapes = {}
        for name, shape in self.observation_features.items():
            if not isinstance(shape, tuple):
                continue
            if (
                len(shape) != 3 or shape[2] != 3
                or any(isinstance(size, bool) or not isinstance(size, int) or size <= 0 for size in shape)
            ):
                raise ValueError(f"Recording camera {name} requires an RGB (H, W, 3) shape")
            self._shapes[name] = shape
        self._image_names = frozenset(self._shapes)
        self._ctx = mp.get_context("spawn")
        self._process = None
        self._pipe = None
        self._lock = threading.RLock()
        self._close_lock = threading.Lock()
        self._closing = threading.Event()
        self._request = 0
        self._episode = 0
        self._active_episode = None
        self._broken: str | None = None
        self._closed = False
        # Replace the tuple atomically; readers must not take the RPC lock,
        # which remains held throughout a potentially long conversion.
        self._progress = (time.monotonic(), "created", 0, None)

    @property
    def last_progress_at(self) -> float:
        return self._progress[0]

    def _note_progress(self, stage, frames=0, total=None) -> None:
        self._progress = (time.monotonic(), stage, frames, total)

    def _acquire_while_progressing(self, lock) -> bool:
        """Wait for an operation to release its lock while it makes progress."""
        seen = self.last_progress_at
        deadline = time.monotonic() + self._timeout
        while True:
            if lock.acquire(timeout=min(0.1, max(0.0, deadline - time.monotonic()))):
                return True
            latest = self.last_progress_at
            if latest > seen:
                seen = latest
                deadline = latest + self._timeout
            if time.monotonic() >= deadline:
                return False

    def start(self) -> None:
        """Spawn from a dedicated thread so only the child inherits its CPU mask."""
        with self._lock:
            if self._closed or self._closing.is_set():
                raise RuntimeError("Recording process has been closed")
            if self._process is not None:
                self._check_usable()
                return
            child = None
            launcher = None
            try:
                self._pipe, child = self._ctx.Pipe()
                self._process = self._ctx.Process(
                    target=_recording_worker,
                    args=(
                        self.config, self.observation_features, self.action_features, child,
                    ),
                    name="onero-recording", daemon=False,
                )
                errors = []

                def launch():
                    try:
                        if self._cpu_affinity is not None:
                            if not hasattr(os, "sched_setaffinity"):
                                raise RuntimeError("Recording CPU affinity requires OS affinity support")
                            requested = set(self._cpu_affinity)
                            os.sched_setaffinity(0, requested)
                            if os.sched_getaffinity(0) != requested:
                                raise RuntimeError("Could not apply the requested recording CPU affinity")
                        self._process.start()
                    except BaseException as exc:
                        errors.append(exc)

                launcher = threading.Thread(target=launch, name="onero-recording-launch")
                launcher.start()
                # Wait only for OS process creation, not child imports or sink
                # initialization. Process.start itself has no timeout API.
                launcher.join()
                if errors:
                    raise RuntimeError(f"Could not launch recording process: {_error_text(errors[0])}") from errors[0]
                logger.info(
                    "Recording process PID=%d launched; capture to disk, encode after stop; CPU affinity=%s",
                    self._process.pid, self._cpu_affinity or "inherited",
                )
            except BaseException:
                self._closed = True
                # Finish an interrupted process launch before closing its pipe.
                if launcher is not None and launcher.is_alive():
                    launcher.join()
                try:
                    if self._process is not None and self._process.pid is not None:
                        self._terminate()
                finally:
                    self._release_transport()
                raise
            finally:
                if child is not None:
                    child.close()

    def _check_usable(self) -> None:
        if self._closed or self._closing.is_set():
            raise RuntimeError("Recording process has been closed")
        if self._broken is not None:
            raise RuntimeError(self._broken)
        if self._process is None or self._process.pid is None:
            raise RuntimeError("Start the recording process before opening an episode")
        if not self._process.is_alive():
            self._broken = (
                f"Recording process PID={self._process.pid} exited (exitcode={self._process.exitcode})"
            )
            raise RuntimeError(self._broken)

    def _rpc(self, operation: str, payload=None, *, timeout: float | None = None, closing=False):
        if not closing:
            self._check_usable()
        if self._pipe is None:
            raise RuntimeError("Recording process pipe is unavailable")
        self._request += 1
        request = self._request
        idle_timeout = self._timeout if timeout is None else timeout
        self._note_progress(f"{operation}: started")
        deadline = time.monotonic() + idle_timeout
        try:
            self._pipe.send((request, operation, payload))
            while True:
                remaining = deadline - time.monotonic()
                # Drain a queued progress/result message before declaring an
                # idle timeout, even if the parent was briefly descheduled.
                if self._pipe.poll(min(0.1, max(0.0, remaining))):
                    received, ok, value = self._pipe.recv()
                    if received != request:
                        if closing and received < request:
                            continue  # Drain a response from an operation that timed out.
                        self._broken = f"Recording IPC response mismatch during {operation}"
                        raise RuntimeError(self._broken)
                    if ok == "progress":
                        if operation not in ("finish", "shutdown"):
                            self._broken = f"Unexpected recording progress during {operation}"
                            raise RuntimeError(self._broken)
                        self._note_progress(*value)
                        deadline = self.last_progress_at + idle_timeout
                        continue
                    if not ok:
                        raise RuntimeError(f"Recording process {operation} failed: {value}")
                    self._note_progress(f"{operation}: complete")
                    return value
                if not self._process.is_alive():
                    self._broken = (
                        f"Recording process PID={self._process.pid} exited during {operation} "
                        f"(exitcode={self._process.exitcode})"
                    )
                    raise RuntimeError(self._broken)
                if time.monotonic() >= deadline:
                    self._broken = (
                        f"Recording process PID={self._process.pid} made no progress for "
                        f"{idle_timeout:g}s during {operation} (last={self._progress[1:]}); "
                        "capture source is retained if conversion is incomplete"
                    )
                    # A genuinely stalled child must not keep converting in
                    # the background after the parent has reported failure.
                    self._terminate()
                    raise RuntimeError(self._broken)
        except (BrokenPipeError, EOFError, OSError) as exc:
            self._broken = (
                f"Recording process PID={self._process.pid} connection lost during {operation}: "
                f"{_error_text(exc)}"
            )
            raise RuntimeError(self._broken) from exc

    def open_episode(self) -> RemoteEpisodeSink:
        with self._lock:
            self._check_usable()
            if self._active_episode is not None:
                raise RuntimeError("A recording episode is already open")
            self._rpc("open")
            self._episode += 1
            self._active_episode = self._episode
            return RemoteEpisodeSink(self, self._episode)

    def _check_episode(self, episode: int) -> None:
        self._check_usable()
        if self._active_episode != episode:
            raise RuntimeError("Recording episode is no longer active")

    def _add(self, episode: int, observation: dict, action: dict, timestamp: float) -> None:
        with self._lock:
            self._check_episode(episode)
            # Float conversion also prevents an unexpected tensor/image from
            # being implicitly serialized in a supposedly small metadata dict.
            values = {
                name: float(value) for name, value in observation.items()
                if name not in self._image_names
            }
            actions = {name: float(value) for name, value in action.items()}
            from .recording_image import pack_recording_image

            values.update({
                name: pack_recording_image(observation[name], shape)
                for name, shape in self._shapes.items()
            })
            # One bounded IPC group at a time; ACK means captured to disk.
            self._rpc("add", (values, actions, float(timestamp)))

    def _finish(self, episode: int) -> None:
        with self._lock:
            self._check_episode(episode)
            try:
                self._rpc("finish")
            finally:
                self._active_episode = None

    def _terminate(self) -> None:
        """Escalate shutdown only for the recorder process owned by this object."""
        if self._process.is_alive():
            logger.error("Terminating unresponsive recording process PID=%d", self._process.pid)
            self._process.terminate()
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=2.0)
        if self._process.is_alive():
            raise RuntimeError(f"Recording process PID={self._process.pid} did not exit after termination")

    def _release_transport(self) -> None:
        if self._pipe is not None:
            self._pipe.close()
            self._pipe = None
        if self._process is not None and not self._process.is_alive():
            self._process.close()
            self._process = None

    def _cleanup_when_idle(self) -> None:
        """Do not close the pipe while a stalled handoff still uses it."""
        with self._lock:
            try:
                # Also handles a launch that completed after close timed out.
                if self._process is not None and self._process.pid is not None:
                    self._terminate()
            except Exception:
                logger.exception("Deferred recording process shutdown failed")
            finally:
                self._active_episode = None
                try:
                    self._release_transport()
                finally:
                    self._closed = True

    def close(self) -> None:
        """Finalize after arm shutdown, waiting as long as work progresses.

        Termination adds at most four seconds and a busy parent writer gets a
        final two-second grace period. Defer closing the pipe if that writer
        still holds the transport lock. Unconverted source files stay on disk.
        """
        if not self._acquire_while_progressing(self._close_lock):
            raise RuntimeError("Recording process shutdown is already in progress")
        acquired = False
        cleanup_deferred = False
        termination_attempted = False
        error = None
        try:
            if self._closed:
                return
            self._closing.set()
            acquired = self._acquire_while_progressing(self._lock)
            if not acquired:
                error = RuntimeError("Recording shutdown timed out waiting for an in-flight writer operation")
                try:
                    if self._process is not None and self._process.pid is not None:
                        termination_attempted = True
                        self._terminate()
                except Exception as exc:
                    error = RuntimeError(f"{error}; {_error_text(exc)}")
                acquired = self._lock.acquire(timeout=2.0)
                if not acquired:
                    threading.Thread(
                        target=self._cleanup_when_idle, name="onero-recording-cleanup", daemon=True,
                    ).start()
                    cleanup_deferred = True
                    raise RuntimeError(
                        f"{error}; IPC cleanup is pending until the parent writer releases its lock"
                    )
            if self._process is not None and self._process.pid is not None:
                if self._process.is_alive() and self._broken is not None:
                    error = error or RuntimeError(self._broken)
                    self._terminate()
                    termination_attempted = True
                elif self._process.is_alive():
                    try:
                        self._rpc("shutdown", closing=True)
                    except Exception as exc:
                        error = error or exc
                elif not self._process.is_alive() and self._process.exitcode != 0:
                    error = error or RuntimeError(
                        f"Recording process exited before shutdown (exitcode={self._process.exitcode})"
                    )
                self._process.join(timeout=self._timeout)
                if self._process.is_alive():
                    error = error or RuntimeError("Recording process shutdown timed out; episode may be incomplete")
                    if not termination_attempted:
                        self._terminate()
            if error is not None:
                raise error
        finally:
            try:
                if acquired:
                    try:
                        self._active_episode = None
                        self._release_transport()
                    finally:
                        self._closed = True
                        self._lock.release()
                elif not cleanup_deferred and self._closing.is_set() and not self._closed:
                    logger.error("Recording cleanup could not acquire the writer lock; IPC retained")
            finally:
                self._close_lock.release()
