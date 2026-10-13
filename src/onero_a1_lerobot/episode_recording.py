"""Bounded disk capture, followed by LeRobot video generation after stop."""

from __future__ import annotations

import logging
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from .sdk_device_config import positive

logger = logging.getLogger(__name__)


@dataclass
class RecordingConfig:
    enabled: bool = False
    repo_id: str | None = None
    task: str | None = None
    root: str | None = None  # exact dataset directory, same semantics as LeRobot
    fps: int = 30
    queue_size: int = 300
    max_queue_bytes: int = 256 * 1024 * 1024  # Queued/in-flight payloads plus reserved IPC copies
    finalize_timeout_s: float = 300.0  # Maximum idle time without reported capture/conversion progress.
    video_encoder: str = "auto"  # Detect Rockchip system FFmpeg; otherwise use PyAV/libx264.

    def validate(self, *, starting: bool = False) -> None:
        if not isinstance(self.enabled, bool):
            raise ValueError("recording.enabled must be a boolean")
        positive(self.fps, "recording.fps")
        if (
            isinstance(self.fps, bool)
            or isinstance(self.queue_size, bool)
            or not isinstance(self.fps, int)
            or not isinstance(self.queue_size, int)
            or self.queue_size < 1
        ):
            raise ValueError("recording fps and queue_size must be positive integers")
        if (
            isinstance(self.max_queue_bytes, bool)
            or not isinstance(self.max_queue_bytes, int)
            or self.max_queue_bytes < 1
        ):
            raise ValueError("recording.max_queue_bytes must be a positive integer")
        positive(self.finalize_timeout_s, "recording.finalize_timeout_s")
        if self.video_encoder not in ("auto", "libx264", "h264_rkmpp"):
            raise ValueError("recording.video_encoder must be auto, libx264 or h264_rkmpp")
        if starting or self.enabled:
            parts = (self.repo_id or "").split("/")
            if len(parts) != 2 or any(p in ("", ".", "..") or "\\" in p for p in parts):
                raise ValueError("Recording requires repo_id in owner/dataset form")
            if not self.task or not self.task.strip():
                raise ValueError("Recording requires a task description")


def build_rgb_encoder(config: RecordingConfig, backend):
    """Return the RGB encoder settings for a resolved video backend.

    These values are the portable contract of the project's own pipeline. The
    native ``lerobot-record`` configuration in
    ``configs/native_dual_joint_follow_record.json`` mirrors them under
    ``dataset.rgb_encoder`` so both recording paths persist the same H.264
    parameters; keep the two in sync (``tests/test_native_record_parity.py``
    fails if they drift).
    """
    from lerobot.configs.video import RGBEncoderConfig

    if backend.name == "h264_rkmpp":
        # LeRobot stores the portable output format. Hardware-specific
        # settings are recorded separately in each feature's info.
        return RGBEncoderConfig(vcodec="h264", preset=None, crf=None, g=config.fps)
    return RGBEncoderConfig(
        vcodec="h264", preset="ultrafast", crf=23, g=config.fps,
        extra_options={"tune": "zerolatency", "bf": "0"},
    )


class LeRobotEpisodeSink:
    """Used only for post-record conversion inside the recording process."""

    def __init__(self, config: RecordingConfig, observation_features: dict, action_features: dict):
        from .dataset import _import_lerobot_dataset_tools

        Dataset, self._build_frame, to_features = _import_lerobot_dataset_tools()
        from lerobot.utils.constants import DEFAULT_FEATURES

        camera_names = tuple(
            name for name, feature in observation_features.items() if isinstance(feature, tuple)
        )
        self._camera_metadata = {
            f"camera.{name}.{field}": {
                "dtype": "float64", "shape": (1,), "names": [field],
            }
            for name in camera_names
            for field in ("time_s", "sequence")
        }
        # Camera acquisition metadata must not become policy observation.state.
        observation_features = {
            name: feature for name, feature in observation_features.items()
            if name not in self._camera_metadata
        }
        self.features = {
            **to_features(observation_features, "observation", use_video=bool(camera_names)),
            **to_features(action_features, "action"),
            **self._camera_metadata,
        }
        # Preserve actual sample time even when control/recording rates differ.
        self.features["control.time_s"] = {
            "dtype": "float64",
            "shape": (1,),
            "names": ["monotonic_time"],
        }
        self.config = config
        self.frames = 0
        video_options = {}
        expected_video_info = {}
        self._has_video = bool(camera_names)
        if self._has_video:
            from .ffmpeg_encoder import resolve_video_backend
            from .streaming_recording import make_streaming_dataset_class

            # Encode directly instead of writing and rereading every RGB PNG.
            # Our encoder consumes a complete camera group before add returns;
            # it never silently drops frames or reuses the shared slot early.
            backend = resolve_video_backend(config.video_encoder)
            Dataset = make_streaming_dataset_class(Dataset, backend)
            rgb_encoder = build_rgb_encoder(config, backend)
            video_options = {
                "rgb_encoder": rgb_encoder,
                "encoder_threads": 1,
                "streaming_encoding": True,
            }
        root = (
            Path(config.root).expanduser()
            if config.root
            else Path.home() / "lerobot_datasets" / config.repo_id
        )
        if root.exists():
            self.dataset = Dataset.resume(
                config.repo_id, root=str(root), video_backend="pyav", **video_options,
            )
            try:
                existing = {
                    k: v for k, v in self.dataset.features.items() if k not in DEFAULT_FEATURES
                }

                def signature(feature):
                    return (
                        feature.get("dtype"),
                        tuple(feature.get("shape", ())),
                        tuple(feature.get("names") or ()),
                    )

                if set(existing) != set(self.features) or any(
                    signature(existing[k]) != signature(f) for k, f in self.features.items()
                ):
                    raise ValueError(
                        "Dataset schema differs from this session; choose another repo_id/root"
                    )
                if self.dataset.fps != config.fps:
                    raise ValueError("Dataset fps differs from recording.fps")
                if self._has_video and self.dataset.meta.total_episodes:
                    expected_encoding = {
                        "video.codec": "h264",
                        "video.g": config.fps, "video.crf": rgb_encoder.crf,
                        "video.preset": rgb_encoder.preset,
                        "video.extra_options": rgb_encoder.extra_options,
                    }
                    for name in camera_names:
                        video_key = f"observation.images.{name}"
                        info = existing[video_key].get("info", {})
                        previous_backend = info.get("a1.video_encoder", {"name": "libx264"})
                        if previous_backend != backend.signature:
                            raise ValueError(
                                f"Dataset {video_key} encoder differs: stored={previous_backend!r}, "
                                f"requested={backend.signature!r}; "
                                "use a new repo_id/root to keep existing videos unchanged"
                            )
                        differences = [
                            f"{key}: stored={info.get(key)!r}, requested={value!r}"
                            for key, value in expected_encoding.items() if info.get(key) != value
                        ]
                        # RGBEncoderConfig describes the software encoder. The
                        # external Rockchip encoder can report yuvj420p output;
                        # its RGB input format is not the MP4 pixel format.
                        supported_formats = (
                            ("yuv420p", "yuvj420p") if backend.name == "h264_rkmpp"
                            else (rgb_encoder.pix_fmt,)
                        )
                        if info.get("video.pix_fmt") not in supported_formats:
                            differences.append(
                                f"video.pix_fmt: stored={info.get('video.pix_fmt')!r}, "
                                f"supported={supported_formats!r}"
                            )
                        if differences:
                            raise ValueError(
                                f"Dataset video encoder settings differ for {video_key}: "
                                + "; ".join(differences) + "; "
                                "use a new repo_id/root to keep existing videos unchanged"
                            )
                        # Do not normalize or rewrite the recorded format. The
                        # finished temporary MP4 must match it exactly before
                        # any new dataset rows/videos are committed.
                        expected_video_info[video_key] = dict(info)
            except BaseException:
                self.dataset.finalize()
                raise
        else:
            self.dataset = Dataset.create(
                repo_id=config.repo_id,
                root=str(root),
                fps=config.fps,
                features=self.features,
                robot_type="onero_sdk",
                use_videos=self._has_video,
                video_backend="pyav",
                **video_options,
            )
        if self._has_video:
            try:
                for name in camera_names:
                    self.dataset.meta.features[f"observation.images.{name}"].setdefault("info", {})[
                        "a1.video_encoder"
                    ] = backend.signature
                # LeRobot 0.6.1 delegates this to our overridden encoder
                # factory. Open real episode streams before reporting ready;
                # no dummy images or dataset frames are generated here.
                self._video_encoder = self.dataset.writer._streaming_encoder
                self._video_encoder.prepare_episode(
                    list(self.dataset.meta.video_keys), self.dataset.root,
                    {f"observation.images.{name}": observation_features[name] for name in camera_names},
                    expected_video_info=expected_video_info,
                )
                logger.info(
                    "Post-record video generation: encoder=%s, GOP=%d, cameras=%s",
                    backend.name, config.fps, camera_names,
                )
            except BaseException:
                self.dataset.finalize()
                raise

    def add(self, observation: dict, action: dict, timestamp: float) -> None:
        import numpy as np

        started = time.monotonic()
        frame = {
            **self._build_frame(self.dataset.features, observation, prefix="observation"),
            **self._build_frame(self.dataset.features, action, prefix="action"),
            "control.time_s": np.asarray([timestamp], dtype=np.float64),
            **{
                name: np.asarray([observation[name]], dtype=np.float64)
                for name in self._camera_metadata
            },
            "task": self.config.task,
        }
        self.dataset.add_frame(frame)
        self.frames += 1
        if self.frames <= 3:
            total = time.monotonic() - started
            video = self._video_encoder.last_group_s if self._has_video else 0.0
            logger.info(
                "Dataset frame %d consumed: total=%.3fs video_group=%.3fs other=%.3fs",
                self.frames, total, video, max(0.0, total - video),
            )

    def finish(self, *, progress=None) -> None:
        try:
            if self.frames:
                if self._has_video:
                    if progress is not None:
                        progress("flushing videos", self.frames, self.frames)
                    self._video_encoder.seal_episode(self.frames)
                if progress is not None:
                    progress("writing dataset and statistics", self.frames, self.frames)
                self.dataset.save_episode(parallel_encoding=False)
                if progress is not None:
                    progress("finalizing dataset", self.frames, self.frames)
        finally:
            self.dataset.finalize()


class EpisodeRecorder:
    def __init__(
        self,
        config: RecordingConfig,
        observation_features: dict,
        action_features: dict,
        sink_factory=LeRobotEpisodeSink,
    ):
        config.validate()
        self.config = config
        self.observation_features = observation_features
        self.action_features = action_features
        self._factory = sink_factory
        self._process_writer = None
        self._cpu_affinity: tuple[int, ...] | None = None
        self._transport_bytes = 0
        self._service_error: str | None = None
        self._job = None
        self._lock = threading.Lock()
        self._next_frame = 0.0
        self._image_keys = tuple(
            name for name, feature in observation_features.items() if isinstance(feature, tuple)
        )

    def configure_cpu_affinity(self, cpus: tuple[int, ...] | None) -> None:
        if self._process_writer is not None:
            raise RuntimeError("Configure recording CPUs before preparing the recording process")
        self._cpu_affinity = cpus

    def prepare(self) -> None:
        """Launch the capture process; dataset initialization is deferred to stop."""
        self.config.validate(starting=True)
        if self._factory is not LeRobotEpisodeSink or self._process_writer is not None:
            return
        from .recording_process import RecordingProcess

        writer = RecordingProcess(
            self.config, self.observation_features, self.action_features,
            cpu_affinity=self._cpu_affinity,
        )
        try:
            writer.start()
        except BaseException:
            try:
                writer.close()
            except Exception:
                logger.exception("Recording process cleanup after startup failure did not finish")
            raise
        self._process_writer = writer

    @property
    def status(self) -> str:
        job = self._job
        if job is None:
            return "error" if self._service_error else "off"
        if job.done.is_set():
            return "error" if job.error or self._service_error else "off"
        if job.stop.is_set():
            return "saving"
        return "recording" if job.ready.is_set() else "starting"

    @property
    def error(self) -> str | None:
        return (self._job.error if self._job is not None else None) or self._service_error

    def fail_service(self, reason: str) -> None:
        """Keep initialization failures visible even before an episode exists."""
        with self._lock:
            self._service_error = f"Recording service unavailable: {reason}"
            if self._job is not None and not self._job.done.is_set():
                self._fail(self._job, str(reason))
        logger.error("%s; CAN following continues", self._service_error)

    @property
    def frames(self) -> int:
        return self._job.frames if self._job is not None else 0

    def start(self) -> None:
        self.config.validate(starting=True)
        self.prepare()
        with self._lock:
            if self._job is not None and not self._job.done.is_set():
                raise RuntimeError(
                    "Recording is already active or the previous episode is still saving"
                )
            self._service_error = None
            self._transport_bytes = 0
            job = SimpleNamespace(
                queue=queue.Queue(maxsize=self.config.queue_size),
                stop=threading.Event(),
                ready=threading.Event(),
                done=threading.Event(),
                error=None,
                frames=0,
                pending_bytes=0,
                last_progress_at=time.monotonic(),
            )
            self._job = job
            self._next_frame = 0.0
            job.thread = threading.Thread(
                target=self._write, args=(job,), name="onero_episode_writer", daemon=True
            )
            try:
                job.thread.start()
            except BaseException as exc:
                job.error = f"Could not start recording handoff: {exc}"
                job.stop.set()
                job.done.set()
                raise

    def _wants_frame(self, job, timestamp: float) -> bool:
        return (
            job is not None and job.ready.is_set()
            and not job.stop.is_set() and not job.done.is_set()
            and timestamp >= self._next_frame
        )

    def wants_frame(self, timestamp: float) -> bool:
        """Check sampling readiness before collecting camera cache references."""
        with self._lock:
            return self._wants_frame(self._job, timestamp)

    def _fail(self, job, reason: str) -> None:
        """Called with the recorder lock held; stop accepting new frames."""
        if not job.error:
            job.error = f"Recording stopped: {reason}. Saving the partial episode; control continues."
            logger.error(job.error)
        job.stop.set()

    def fail(self, reason: str) -> None:
        """Stop this episode on a camera/queue error, leaving arm control active."""
        with self._lock:
            if self._job is not None and not self._job.done.is_set():
                self._fail(self._job, str(reason))

    def submit(self, observation: dict, action: dict, timestamp: float) -> None:
        with self._lock:
            job = self._job
            if not self._wants_frame(job, timestamp):
                return
            try:
                image_bytes = sum(observation[key].nbytes for key in self._image_keys)
            except (KeyError, AttributeError) as exc:
                self._fail(job, f"missing or invalid camera image ({exc})")
                return
            # Budget temporary IPC serialization/receive copies as well as
            # the original references. USB capture retains compressed JPEGs;
            # mock/RGB sources are packed only by the handoff thread.
            transport_bytes = max(self._transport_bytes, 4 * image_bytes)
            if job.pending_bytes + image_bytes + transport_bytes > self.config.max_queue_bytes:
                self._fail(
                    job,
                    f"image queue memory limit exceeded: {job.pending_bytes} buffered + "
                    f"{image_bytes} new + {transport_bytes} reserved IPC bytes "
                    f"> recording.max_queue_bytes={self.config.max_queue_bytes} "
                    f"(written={job.frames}, queued={job.queue.qsize()}, target_fps={self.config.fps})",
                )
                return
            self._transport_bytes = transport_bytes
            period = 1 / self.config.fps
            # Keep the sampling phase: at 100 Hz control / 30 Hz recording,
            # alternating 3/4 ticks must average 30 fps, not round down to 25.
            self._next_frame = self._next_frame + period if self._next_frame else timestamp + period
            if self._next_frame <= timestamp:
                self._next_frame = timestamp + period
            try:
                # Shallow copies retain immutable JPEG/RGB references; camera
                # bytes are written to disk by the independent capture worker.
                job.queue.put_nowait((dict(observation), dict(action), timestamp, image_bytes))
                job.pending_bytes += image_bytes
            except queue.Full:
                self._fail(job, "recording queue full")

    def stop(self) -> None:
        with self._lock:
            if self._job is not None:
                self._job.stop.set()

    def _write(self, job) -> None:
        sink = None
        try:
            if job.stop.is_set():
                return
            if self._cpu_affinity is not None and hasattr(os, "sched_setaffinity"):
                os.sched_setaffinity(0, self._cpu_affinity)
            sink = (
                self._process_writer.open_episode()
                if self._process_writer is not None
                else self._factory(self.config, self.observation_features, self.action_features)
            )
            with self._lock:
                cancelled = job.stop.is_set()
                if not cancelled:
                    job.ready.set()
                job.last_progress_at = time.monotonic()
            if cancelled:
                logger.info("Recording preparation completed after stop; no frames accepted")
                return
            logger.info("Recording writer ready: repo=%s fps=%d; capture to disk, video after stop", self.config.repo_id, self.config.fps)
            report_started = time.monotonic()
            report_frames = job.frames
            while not job.stop.is_set() or not job.queue.empty():
                try:
                    observation, action, timestamp, image_bytes = job.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    write_started = time.monotonic()
                    sink.add(observation, action, timestamp)
                    job.frames += 1
                    now = time.monotonic()
                    job.last_progress_at = now
                    if job.frames <= 3:
                        logger.info(
                            "Capture frame %d saved in %.3fs (image IPC + disk journal; no video encoding)",
                            job.frames, now - write_started,
                        )
                    if now - report_started >= 5.0:
                        with self._lock:
                            queued = job.queue.qsize()
                            buffered = job.pending_bytes + self._transport_bytes
                        logger.info(
                            "Capture throughput=%.2f FPS target=%d; queued=%d image_budget=%.1f MiB",
                            (job.frames - report_frames) / (now - report_started),
                            self.config.fps, queued, buffered / (1024 * 1024),
                        )
                        report_started, report_frames = now, job.frames
                finally:
                    # Keep the memory charged until disk capture acknowledges
                    # the group, not merely until the worker dequeues it.
                    del observation, action
                    with self._lock:
                        job.pending_bytes -= image_bytes
        except Exception as exc:
            with self._lock:
                self._fail(job, f"{type(exc).__name__}: {exc}")
            logger.exception("Recording failed; control continues")
        finally:
            with self._lock:
                job.stop.set()
                # A failed sink cannot consume remaining frames. Release their
                # image references before finalization, including error paths.
                while True:
                    try:
                        job.queue.get_nowait()
                    except queue.Empty:
                        break
                job.pending_bytes = 0
            if sink is not None:
                try:
                    job.last_progress_at = time.monotonic()
                    sink.finish()
                except Exception as exc:
                    detail = f"Episode finalization failed: {type(exc).__name__}: {exc}"
                    job.error = f"{job.error} {detail}" if job.error else detail
                    logger.exception("Episode finalization failed")
            job.done.set()
            logger.info(
                "Episode writer finished: frames=%d status=%s",
                job.frames, job.error or ("saved" if job.frames else "empty"),
            )

    def wait(self, timeout: float | None = None) -> None:
        job = self._job
        if job is None:
            return
        # Preserve an explicit caller's absolute wait limit. Normal shutdown
        # uses the configured inactivity limit, not total episode duration.
        if timeout is not None:
            if not job.done.wait(timeout):
                raise TimeoutError(f"Episode writer has not finished after {timeout:g}s")
            return
        idle_timeout = self.config.finalize_timeout_s
        deadline = time.monotonic() + idle_timeout
        seen = 0.0
        while not job.done.wait(min(0.25, max(0.0, deadline - time.monotonic()))):
            latest = job.last_progress_at
            if self._process_writer is not None:
                latest = max(latest, self._process_writer.last_progress_at)
            if latest > seen:
                seen = latest
                deadline = max(deadline, latest + idle_timeout)
            if not job.thread.is_alive():
                raise RuntimeError("Episode writer exited without reporting its final result")
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Episode writer made no progress for {idle_timeout:g}s; "
                    "unfinished capture files are retained for recovery"
                )

    def close(self) -> None:
        self.stop()
        try:
            self.wait()
        except Exception as exc:
            self._record_shutdown_error(exc)
        finally:
            if self._process_writer is not None:
                try:
                    self._process_writer.close()
                except Exception as exc:
                    self._record_shutdown_error(exc)
                finally:
                    self._process_writer = None
                    self._transport_bytes = 0

    def _record_shutdown_error(self, error: Exception) -> None:
        detail = f"Recording shutdown failed: {type(error).__name__}: {error}"
        with self._lock:
            self._service_error = (
                f"{self._service_error}; {detail}" if self._service_error else detail
            )
        # Retain a failed recording's error/nonzero CLI status without raising
        # it again during cleanup and skipping the final session summary.
        logger.error("%s", detail)
