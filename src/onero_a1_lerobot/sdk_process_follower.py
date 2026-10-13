"""Run a follower's complete SDK servo transaction in its own process."""

from __future__ import annotations

import logging
import math
import os
import pickle
import signal
import struct
import threading
import time
from types import SimpleNamespace

from .arm_mapping import joint_vector
from .sdk_client import SdkDevice, _joint_velocities
from .sdk_process_leader import _HEARTBEAT_TIMEOUT, _NO_GRIPPER, ProcessLeaderDevice

logger = logging.getLogger(__name__)


# Keep the frequent servo messages fixed-size; infrequent control RPCs keep the
# existing Connection pickle protocol and error payloads.
_COMMAND_FRAME = struct.Struct("!4sQQB16d")
_COMMAND_ACK = struct.Struct("!4sQQ11d")
_COMMAND_MAGIC = b"FSC1"
_ACK_MAGIC = b"FSA1"
_ZERO_VELOCITY = (0.0,) * 7


class _FrameDecodeError(ValueError):
    def __init__(self, detail, request=0):
        super().__init__(detail)
        self.request = request


def _encode_frame(message):
    request, operation, argument = message
    if operation == "command":
        frame, positions, velocities, submitted, deadline = argument
        has_velocity = velocities is not None
        return _COMMAND_FRAME.pack(
            _COMMAND_MAGIC, request, frame, has_velocity, submitted, deadline,
            *positions, *(velocities if has_velocity else _ZERO_VELOCITY),
        )
    if operation is True and isinstance(argument, tuple) and len(argument) == 3:
        frame, state, timing = argument
        return _COMMAND_ACK.pack(
            _ACK_MAGIC, request, frame, *state.joints, state.timestamp,
            getattr(state, "host_time", state.timestamp), timing["wall_ms"], timing["cpu_ms"],
        )
    return None


def _decode_frame(payload):
    magic = payload[:4]
    if magic in (_COMMAND_MAGIC, _ACK_MAGIC):
        request = struct.unpack_from("!Q", payload, 4)[0] if len(payload) >= 12 else 0
        codec = _COMMAND_FRAME if magic == _COMMAND_MAGIC else _COMMAND_ACK
        if len(payload) != codec.size:
            raise _FrameDecodeError("Invalid follower frame length", request)
        values = codec.unpack(payload)
        if magic == _COMMAND_MAGIC:
            _, request, frame, has_velocity, submitted, deadline, *joints = values
            if has_velocity not in (0, 1) or not all(map(math.isfinite, values[4:])):
                raise _FrameDecodeError("Invalid follower command values", request)
            if deadline < submitted:
                raise _FrameDecodeError("Invalid follower command deadline", request)
            return request, "command", (
                frame, joints[:7], joints[7:] if has_velocity else None, submitted, deadline,
            )
        _, request, frame, *data = values
        if not all(map(math.isfinite, data)) or data[-1] < 0 or data[-2] < 0:
            raise _FrameDecodeError("Invalid follower command acknowledgement", request)
        state = SimpleNamespace(joints=data[:7], timestamp=data[7], host_time=data[8])
        timing = {"frame_id": frame, "wall_ms": data[9], "cpu_ms": data[10]}
        return request, True, (frame, state, timing)
    try:
        return pickle.loads(payload)
    except Exception as exc:
        raise _FrameDecodeError("Invalid follower IPC message") from exc


class _FollowerPipe:
    """Connection-compatible codec; lifecycle/control calls remain unchanged."""

    def __init__(self, connection):
        self._connection = connection

    def send(self, message):
        payload = _encode_frame(message)
        if payload is None:
            self._connection.send(message)
        else:
            self._connection.send_bytes(payload)

    def recv(self):
        return _decode_frame(self._connection.recv_bytes())

    def poll(self, timeout=0):
        return self._connection.poll(timeout)

    def close(self):
        self._connection.close()

    @property
    def closed(self):
        return self._connection.closed


def _follower_worker(config, control_hz, max_gap, pipe, cache, heartbeat, fault,
                     fault_ready, factory):
    pipe = _FollowerPipe(pipe)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", force=True)
    logger.info("Startup: follower worker entered PID=%d channel=%s", os.getpid(), config.channel)
    device = factory(config, simulation_control_hz=control_hz)
    holding = orphaned = False
    stream_stamp = None
    last_frame = None
    next_hold = 0.0
    hold_failure_reported = False

    def latch(detail):
        nonlocal holding
        holding = True
        if not fault_ready.value:
            message = str(detail).encode("utf-8", errors="replace")[:2047]
            fault[:len(message)] = message
            fault[len(message)] = 0
            fault_ready.value = True
            logger.error("Follower %s PID=%d: %s", config.channel, os.getpid(), detail)

    def reply(request, ok, value):
        nonlocal orphaned
        try:
            pipe.send((request, ok, value))
        except (BrokenPipeError, EOFError, OSError):
            orphaned = True
            latch("Parent connection lost; retaining position hold, no automatic disable")

    def publish(state):
        gripper = _NO_GRIPPER if getattr(state, "gripper", None) is None else float(state.gripper)
        with cache.get_lock():
            cache[:] = [float(state.timestamp), *state.joints, gripper]
        return state

    try:
        device.connect(leader=False)
        publish(device.snapshot(1.0))
    except BaseException as exc:
        latch(f"Follower startup failed: {exc}")
        try:
            device.close()
        except BaseException as cleanup_error:
            logger.exception("Follower PID=%d startup cleanup failed", os.getpid())
            reply(0, False, f"{exc}; cleanup failed: {cleanup_error}")
        else:
            reply(0, False, str(exc))
            pipe.close()
            return
    else:
        reply(0, True, None)
        logger.info("Follower %s servo runs in PID=%d", config.channel, os.getpid())

    while True:
        now = time.monotonic()
        if not fault_ready.value:
            if device.gripper_fault is not None:
                # A faulted gripper still answers reads with its last position, so
                # stop following instead of recording a dead channel. Latched like a
                # heartbeat loss, so the arm keeps its position hold.
                latch(device.gripper_fault)
            elif now - heartbeat.value > _HEARTBEAT_TIMEOUT:
                latch("Parent heartbeat expired; retaining position hold, no automatic disable")
            elif stream_stamp is not None and now - stream_stamp > max_gap:
                latch(f"Follower command stream exceeded {max_gap:.3f}s; retaining position hold")
        command = None
        if not orphaned:
            try:
                # A blocking poll wakes immediately for a command, without adding
                # a fixed sleep to every servo cycle.
                wait = max(0.0, min(0.005, next_hold - now)) if holding else 0.005
                command = pipe.recv() if pipe.poll(wait) else None
            except _FrameDecodeError as exc:
                latch(f"Malformed follower IPC: {exc}")
                reply(exc.request, False, str(exc))
            except (EOFError, OSError):
                orphaned = True
                latch("Parent connection lost; retaining position hold, no automatic disable")
        if command is not None:
            request, operation, argument = command
            try:
                if operation == "close":
                    device.close()
                    reply(request, True, None)
                    pipe.close()
                    return
                if operation == "hold":
                    holding = True
                    stream_stamp = None
                    result = device.hold()
                    if result is not True:
                        latch(device.fault_reason or "Follower position hold unavailable")
                    else:
                        publish(device.snapshot(1.0))
                    next_hold = time.monotonic() + 0.005
                    reply(request, True, result is True)
                elif operation == "clear_hold":
                    if fault_ready.value:
                        raise RuntimeError("Follower fault is latched; reconnect required")
                    device.clear_hold()
                    holding = False
                    stream_stamp = None
                    last_frame = None
                    reply(request, True, None)
                elif operation == "gripper":
                    if fault_ready.value or holding:
                        raise RuntimeError(
                            "Follower is holding or faulted; gripper command rejected"
                        )
                    device.gripper_command(argument)
                    # Refresh the shared cache so the next observation reflects
                    # the new gripper target even between joint command frames.
                    publish(device.snapshot(1.0))
                    reply(request, True, None)
                elif operation == "query":
                    if fault_ready.value:
                        raise RuntimeError("Follower fault is latched; reconnect required")
                    reply(request, True, publish(device.query(argument)))
                elif operation == "command":
                    frame, positions, velocities, submitted, deadline = argument
                    now = time.monotonic()
                    if fault_ready.value or holding:
                        raise RuntimeError("Follower is holding or faulted; command rejected")
                    if not math.isfinite(deadline) or now > deadline:
                        raise RuntimeError(f"Follower frame {frame} expired before execution")
                    if last_frame is not None and frame <= last_frame:
                        raise RuntimeError(f"Follower frame {frame} is not newer than {last_frame}")
                    stream_stamp = submitted
                    last_frame = frame
                    cpu_start = time.thread_time()
                    wall_start = time.monotonic()
                    state = device.command(positions, velocities)
                    completed = time.monotonic()
                    timing = {"frame_id": frame, "wall_ms": (completed - wall_start) * 1000,
                              "cpu_ms": (time.thread_time() - cpu_start) * 1000}
                    publish(state)
                    if completed > deadline:
                        raise RuntimeError(f"Follower frame {frame} exceeded its execution deadline")
                    reply(request, True, (frame, state, timing))
                else:
                    raise ValueError(f"Unknown follower operation: {operation}")
            except BaseException as exc:
                latch(f"Follower {operation} failed: {exc}")
                reply(request, False, str(exc))
        if holding and time.monotonic() >= next_hold:
            try:
                if device.hold() is not True:
                    raise RuntimeError(device.fault_reason or "Follower position hold unavailable")
                publish(device.snapshot(1.0))
            except BaseException as exc:
                latch(str(exc))
                if not hold_failure_reported:
                    logger.error("Follower PID=%d position hold unavailable: %s", os.getpid(), exc)
                    hold_failure_reported = True
            next_hold = time.monotonic() + 0.005
        if orphaned:
            time.sleep(0.005)


class ProcessFollowerDevice(ProcessLeaderDevice):
    """One command in flight, acknowledged only after the SDK receives feedback."""

    _role = "Follower"
    _leader_role = False

    def __init__(self, config, *, simulation_control_hz=200, max_control_gap_s=0.25,
                 _device_factory=SdkDevice):
        if not math.isfinite(max_control_gap_s) or max_control_gap_s <= 0:
            raise ValueError("max_control_gap_s must be finite and positive")
        super().__init__(config, simulation_control_hz=simulation_control_hz,
                         _device_factory=_device_factory)
        self._rpc_lock = threading.RLock()
        self._max_gap = max_control_gap_s
        self._pending_command = None
        self.last_command_timing = None

    @property
    def limits(self):
        return self._metadata.limits

    @property
    def max_joint_velocity(self):
        return self._metadata.max_joint_velocity

    @property
    def joint_velocity_limits(self):
        return self._metadata.joint_velocity_limits

    @property
    def servo_max_step(self):
        return self._metadata.servo_max_step

    @property
    def command_pending(self):
        return self._pending_command is not None

    def _worker_spec(self, damping, child):
        self._pipe = _FollowerPipe(self._pipe)
        return _follower_worker, (
            self.config, self._control_hz, self._max_gap, child, self._cache,
            self._heartbeat, self._fault, self._fault_ready, self._factory,
        )

    def connect(self, *, leader=False, damping=0.3):
        super().connect(leader=leader, damping=damping)
        self._pending_command = None
        self.last_command_timing = None

    def begin_command(self, positions, velocities=None, *, frame_id):
        q = joint_vector(positions).tolist()
        velocity = _joint_velocities(velocities)
        if (isinstance(frame_id, bool) or not isinstance(frame_id, int)
                or not 0 <= frame_id < 2**64):
            raise ValueError("frame_id must be an unsigned 64-bit integer")
        with self._rpc_lock:
            if self._pending_command is not None:
                raise RuntimeError("A follower command is already in flight")
            if self.fault_reason:
                raise RuntimeError(self.fault_reason)
            if not self.connected:
                raise ConnectionError("Follower is not connected")
            submitted = time.monotonic()
            deadline = submitted + self._max_gap
            self._request += 1
            try:
                self._pipe.send((self._request, "command", (frame_id, q, velocity, submitted, deadline)))
            except (BrokenPipeError, EOFError, OSError) as exc:
                self._local_fault = f"Follower PID={self._process.pid} connection lost"
                raise RuntimeError(self._local_fault) from exc
            self._pending_command = self._request, frame_id, deadline

    def _finish_command_locked(self, timeout):
        if self._pending_command is None:
            raise RuntimeError("No follower command is in flight")
        request, expected_frame, _deadline = self._pending_command
        try:
            frame, state, timing = self._receive(request, timeout)
        except TimeoutError:
            # Retain the pending request for hold/close to drain its eventual ACK.
            raise
        except BaseException:
            self._pending_command = None
            raise
        self._pending_command = None
        if frame != expected_frame or timing.get("frame_id") != expected_frame:
            self._local_fault = f"Follower frame mismatch: expected {expected_frame}, got {frame}"
            raise RuntimeError(self._local_fault)
        self.last_command_timing = timing
        return state

    def finish_command(self):
        with self._rpc_lock:
            if self._pending_command is None:
                raise RuntimeError("No follower command is in flight")
            remaining = self._pending_command[2] - time.monotonic()
            # A completed command may already be buffered even if collecting the
            # other arm's result delayed the caller beyond this frame's deadline.
            return self._finish_command_locked(max(0.001, remaining))

    def command(self, positions, velocities=None):
        self.begin_command(positions, velocities, frame_id=self._request + 1)
        return self.finish_command()

    def _rpc(self, operation, argument=None):
        # Never let a late motion response become the response to a hold/close.
        # Child operations are sequential, so no queued target can run after hold.
        with self._rpc_lock:
            if self._pending_command is not None:
                try:
                    self._finish_command_locked(15.0)
                # TimeoutError is not a RuntimeError; without it a late command
                # would stop hold/close from ever reaching the child, which is
                # the opposite of what _finish_command_locked promises.
                except (RuntimeError, TimeoutError):
                    if operation not in ("hold", "close"):
                        raise
            return super()._rpc(operation, argument)

    def query(self, stale_s):
        return self._rpc("query", stale_s)

    def gripper_command(self, position):
        self._rpc("gripper", float(position))

    def clear_hold(self):
        if self.fault_reason:
            raise RuntimeError(self.fault_reason)
        self._rpc("clear_hold")
        self.last_command_timing = None
