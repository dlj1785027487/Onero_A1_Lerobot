"""Keep CAN freedrive and its Python threads out of the follower process."""

from __future__ import annotations

import atexit
import logging
import math
import multiprocessing as mp
import os
import signal
import threading
import time
from types import SimpleNamespace

from .sdk_client import SdkDevice

logger = logging.getLogger(__name__)
_HEARTBEAT_TIMEOUT = 5.0
# Shared cache layout: [timestamp, *7 joints, gripper]. A gripper value below
# zero means the arm has no gripper channel.
_NO_GRIPPER = -1.0


def _leader_worker(config, control_hz, damping, pipe, cache, heartbeat, fault,
                   fault_ready, factory):
    # The parent handles terminal interrupts and explicitly requests torque-off.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", force=True)
    logger.info("Startup: leader worker entered PID=%d channel=%s", os.getpid(), config.channel)
    device = factory(config, simulation_control_hz=control_hz)
    holding = orphaned = False

    def latch(detail):
        if not fault_ready.value:
            message = str(detail).encode("utf-8", errors="replace")[:2047]
            fault[:len(message)] = message
            fault[len(message)] = 0
            fault_ready.value = True
            logger.error("Leader %s PID=%d: %s", config.channel, os.getpid(), detail)

    def reply(request, ok, value):
        nonlocal orphaned, holding
        try:
            pipe.send((request, ok, value))
        except (BrokenPipeError, EOFError, OSError):
            orphaned = holding = True
            latch("Parent connection lost; retaining position hold, no automatic disable")

    def publish():
        state = device.snapshot(1.0)
        gripper = _NO_GRIPPER if state.gripper is None else float(state.gripper)
        with cache.get_lock():
            cache[:] = [float(state.timestamp), *state.joints, gripper]

    started = False
    try:
        device.connect(leader=True, damping=damping)
        publish()
        started = True
    except BaseException as exc:
        latch(f"Leader startup failed: {exc}")
        try:
            device.close()
        except BaseException as cleanup_error:
            latch(f"Leader startup cleanup failed: {cleanup_error}")
            logger.exception("Leader PID=%d startup cleanup failed", os.getpid())
            reply(0, False, f"{exc}; cleanup failed: {cleanup_error}")
            # Keep reading the pipe: the parent still owns this process and its
            # explicit close is the only way to release the arm cleanly. Only a
            # failed reply above (parent really gone) may mark it orphaned.
            holding = True
        else:
            reply(0, False, str(exc))
            pipe.close()
            return
    if started:
        reply(0, True, None)
    logger.info("Leader %s freedrive runs in PID=%d", config.channel, os.getpid())
    hold_failure_reported = False
    while True:
        if not orphaned and not fault_ready.value and time.monotonic() - heartbeat.value > _HEARTBEAT_TIMEOUT:
            holding = True
            latch("Parent heartbeat expired; retaining position hold, no automatic disable")
        if not orphaned:
            try:
                command = pipe.recv() if pipe.poll(0) else None
            except (EOFError, OSError):
                command = None
                orphaned = holding = True
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
                        result = device.hold()
                        if result is not True:
                            latch(device.fault_reason or "Leader position hold unavailable")
                        reply(request, True, result is True)
                    elif operation == "resume":
                        if fault_ready.value:
                            raise RuntimeError("Leader fault is latched; reconnect required")
                        device.resume_leader(argument)
                        holding = False
                        reply(request, True, None)
                    else:
                        raise ValueError(f"Unknown leader operation: {operation}")
                except BaseException as exc:
                    holding = True
                    latch(f"Leader {operation} failed: {exc}")
                    reply(request, False, str(exc))
        try:
            if holding:
                if device.hold() is not True:
                    raise RuntimeError(device.fault_reason or "Leader position hold unavailable")
            # A faulted leader gripper still reports a position, so stop before
            # mirroring it into the follower. This latches through the handler
            # below, which keeps the leader arm holding instead of going limp.
            if device.gripper_fault is not None:
                raise RuntimeError(device.gripper_fault)
            if not fault_ready.value:
                publish()
        except Exception as exc:
            holding = True
            latch(str(exc))
            if not hold_failure_reported:
                logger.error("Leader PID=%d fault; requesting hold: %s", os.getpid(), exc)
                hold_failure_reported = True
        # Refreshes the cache/held target; SDK freedrive retains its own 200 Hz loop.
        time.sleep(0.005)


class ProcessLeaderDevice:
    """Leader-only SdkDevice proxy; all Arm/CAN handles are created after spawn."""

    _role = "Leader"
    _leader_role = True

    def __init__(self, config, *, simulation_control_hz=200, _device_factory=SdkDevice):
        self.config = config
        self._control_hz = simulation_control_hz
        self._factory = _device_factory
        self._metadata = SdkDevice(config, simulation_control_hz=simulation_control_hz)
        self._ctx = mp.get_context("spawn")
        self._cache = self._ctx.Array("d", 9)
        self._heartbeat = self._ctx.RawValue("d", 0.0)
        self._fault = self._ctx.RawArray("B", 2048)
        self._fault_ready = self._ctx.RawValue("b", False)
        self._process = self._pipe = self._heartbeat_thread = None
        self._heartbeat_stop = threading.Event()
        self._rpc_lock = threading.Lock()
        self._request = 0
        self._close_request = None
        self._close_ack = False
        self._local_fault = None
        self._connected = False

    def preflight(self):
        self._metadata.preflight()

    @property
    def connected(self):
        # Retain ownership after a crash so the session reports HOLD UNAVAILABLE.
        return self._connected

    @property
    def fault_reason(self):
        if self._fault_ready.value:
            return bytes(self._fault[:]).split(b"\0", 1)[0].decode("utf-8", errors="replace")
        if self._local_fault:
            return self._local_fault
        if self._connected and not self._process.is_alive():
            return f"{self._role} process PID={self._process.pid} exited ({self._process.exitcode})"
        return None

    def _beat(self):
        while not self._heartbeat_stop.is_set():
            self._heartbeat.value = time.monotonic()
            self._heartbeat_stop.wait(0.1)

    def _worker_spec(self, damping, child):
        return _leader_worker, (
            self.config, self._control_hz, damping, child, self._cache,
            self._heartbeat, self._fault, self._fault_ready, self._factory,
        )

    def connect(self, *, leader=True, damping=0.3):
        if leader != self._leader_role:
            raise ValueError(f"{type(self).__name__} does not support leader={leader}")
        if self._process is not None:
            raise RuntimeError(f"Close the previous {self._role.lower()} process before reconnecting")
        started = time.monotonic()
        logger.info("Startup: spawning %s worker channel=%s", self._role.lower(), self.config.channel)
        self.preflight()
        self._cache[:] = [0.0] * 9
        self._fault_ready.value = False
        self._local_fault = None
        self._close_request = None
        self._close_ack = False
        self._heartbeat_stop.clear()
        self._heartbeat.value = time.monotonic()
        parent, child = self._ctx.Pipe()
        self._pipe = parent
        target, args = self._worker_spec(damping, child)
        self._process = self._ctx.Process(
            target=target, args=args,
            name=f"{self._role.lower()}-{self.config.channel}", daemon=False,
        )
        self._process.start()
        child.close()
        atexit.register(self._abandon)
        self._heartbeat_thread = threading.Thread(target=self._beat, daemon=True,
                                                   name=f"heartbeat-{self._process.pid}")
        self._heartbeat_thread.start()
        self._receive(0, 30.0)
        self._connected = True
        logger.info(
            "Startup: %s worker PID=%d connected in %.3fs", self._role.lower(),
            self._process.pid, time.monotonic() - started,
        )

    def _receive(self, request, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._pipe.poll(min(0.1, max(0.0, deadline - time.monotonic()))):
                try:
                    received, ok, value = self._pipe.recv()
                except (EOFError, OSError) as exc:
                    raise RuntimeError(f"{self._role} PID={self._process.pid} connection lost") from exc
                if received != request:
                    continue  # A timed-out previous request may have completed late.
                if not ok:
                    if request == self._close_request:
                        self._close_request = None
                    raise RuntimeError(f"{self._role} PID={self._process.pid}: {value}")
                return value
            if not self._process.is_alive():
                raise RuntimeError(f"{self._role} PID={self._process.pid} exited unexpectedly")
        self._local_fault = f"{self._role} PID={self._process.pid} request {request} timed out"
        raise TimeoutError(self._local_fault)

    def _rpc(self, operation, argument=None):
        with self._rpc_lock:
            if operation == "close" and self._close_request is not None:
                return self._receive(self._close_request, 15.0)
            if self._process is None or not self._process.is_alive():
                raise RuntimeError(self.fault_reason or f"{self._role} is not connected")
            self._request += 1
            try:
                self._pipe.send((self._request, operation, argument))
            except (BrokenPipeError, EOFError, OSError) as exc:
                self._local_fault = f"{self._role} PID={self._process.pid} connection lost"
                raise RuntimeError(self._local_fault) from exc
            if operation == "close":
                self._close_request = self._request
            return self._receive(self._request, 15.0)

    def snapshot(self, stale_s):
        if self.fault_reason:
            raise RuntimeError(self.fault_reason)
        if not self.connected:
            raise ConnectionError(f"{self._role} is not connected")
        lock = self._cache.get_lock()
        if not lock.acquire(timeout=0.02):
            self._local_fault = f"{self._role} shared state lock timed out"
            raise RuntimeError(self._local_fault)
        try:
            stamp, *values = self._cache[:]
            *joints, raw_gripper = values
        finally:
            lock.release()
        now = time.monotonic()
        age = now - stamp
        if not all(math.isfinite(value) for value in [stamp, *joints]):
            self._local_fault = f"Invalid {self._role.lower()} shared state"
            raise RuntimeError(self._local_fault)
        if age < -0.01 or age > stale_s:
            self._local_fault = f"SDK state is stale (age={age:.3f}s)"
            raise RuntimeError(self._local_fault)
        gripper = None if raw_gripper < 0.0 else raw_gripper
        return SimpleNamespace(
            joints=joints, timestamp=stamp, host_time=now, gripper=gripper,
        )

    def hold(self):
        if self._process is None:
            return False
        return self._rpc("hold")

    def resume_leader(self, damping):
        if self.fault_reason:
            raise RuntimeError(self.fault_reason)
        self._rpc("resume", damping)

    def step_simulation_leader(self):
        pass

    def _abandon(self):
        self._heartbeat_stop.set()
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=1.0)
        if self._pipe is not None:
            self._pipe.close()
        if self._process is not None and self._process.is_alive():
            logger.critical("%s PID=%d remains alive for hold; manual cleanup required", self._role, self._process.pid)

    def close(self):
        if self._process is None:
            return
        try:
            if self._close_ack:
                pass
            elif self._process.is_alive() or self._close_request is not None:
                self._rpc("close")
                self._close_ack = True
            elif self._connected:
                raise RuntimeError(self.fault_reason or "Leader exited before explicit disable")
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                raise RuntimeError(f"{self._role} PID={self._process.pid} did not exit after close")
            if self._process.exitcode != 0:
                raise RuntimeError(f"{self._role} PID={self._process.pid} exit={self._process.exitcode}")
            self._process = None
            self._connected = False
        except BaseException:
            logger.exception("%s PID=%d close failed; hold retained where possible", self._role, self._process.pid)
            # Keep the channel available for an explicit close retry. Atexit
            # closes it and requests orphan hold if the caller gives up.
            raise
        self._abandon()
        atexit.unregister(self._abandon)
