"""Offline regression tests for the fault / shutdown paths.

These pin behaviour that only shows up on real hardware, so they are built out
of real pipes, real threads and stand-in devices instead of CAN:

* the leader worker must still answer ``close`` after a failed startup cleanup
  (an orphaned worker can never be closed again and needs ``kill -9``),
* ``ProcessFollowerDevice`` must still deliver ``hold``/``close`` when a late
  command times out,
* ``SdkSession.close()`` must run the torque-off chain even if the position hold
  fails,
* a USB camera whose worker thread has exited must be reconnectable.
"""

from __future__ import annotations

import multiprocessing
import signal
import threading
import time
from types import SimpleNamespace

import pytest

from onero_a1_lerobot.sdk_session import SdkSession

# --------------------------------------------------------------------------- #
# leader worker: a failed startup cleanup must not turn it into a dead process
# --------------------------------------------------------------------------- #


class _FakeLeaderDevice:
    def __init__(self, *_args, **_kwargs) -> None:
        self.closes = 0
        self.fault_reason = None
        self.gripper_fault = None

    def connect(self, **_kwargs) -> None:
        raise RuntimeError("boom")

    def close(self) -> None:
        self.closes += 1
        if self.closes <= self._fail_close_times:
            raise RuntimeError("cleanup boom")

    def snapshot(self, _stale_s):
        return SimpleNamespace(timestamp=time.monotonic(), joints=[0.0] * 7, gripper=None)

    def hold(self) -> bool:
        return True

    def resume_leader(self, _damping) -> None:
        pass


@pytest.fixture
def leader_pipe(monkeypatch):
    """Spawn the real leader worker in-process, with SIGINT handling disabled."""
    # _leader_worker ignores SIGINT as its first act; doing that in-process would
    # leave the test session unable to be interrupted.
    monkeypatch.setattr(signal, "signal", lambda *args, **kwargs: None)
    created: list = []

    def start(fail_close_times: int):
        from onero_a1_lerobot import sdk_process_leader

        ctx = multiprocessing.get_context("spawn")
        parent, child = ctx.Pipe()
        fake_cls = type(
            "FakeLeaderDevice",
            (_FakeLeaderDevice,),
            {"_fail_close_times": fail_close_times},
        )
        created.append(child)
        thread = threading.Thread(
            target=sdk_process_leader._leader_worker,
            args=(
                SimpleNamespace(channel="canX"),
                200.0,
                0.3,
                child,
                ctx.Array("d", 9),
                ctx.RawValue("d", time.monotonic()),
                ctx.RawArray("B", 2048),
                ctx.RawValue("b", False),
                fake_cls,
            ),
            daemon=True,
        )
        thread.start()
        return parent, thread

    yield start


def test_leader_worker_answers_exactly_one_startup_reply(leader_pipe) -> None:
    parent, thread = leader_pipe(1)
    assert parent.poll(5.0), "the worker never reported its failed startup"
    request, ok, detail = parent.recv()
    assert (request, ok) == (0, False)
    assert "cleanup failed" in detail
    # The old bug sent a second, bogus success reply after the failure.
    assert not parent.poll(0.3), "a stray reply followed the startup failure"
    # ...and must still be closable, which is the whole point.
    parent.send((99, "close", None))
    assert parent.poll(3.0), "the worker ignored close after a failed cleanup"
    assert parent.recv() == (99, True, None)
    thread.join(2.0)
    assert not thread.is_alive()


def test_leader_worker_reports_close_failure_but_keeps_holding(leader_pipe) -> None:
    """A device that cannot be closed must answer, not go silent."""
    parent, thread = leader_pipe(99)
    assert parent.poll(5.0)
    parent.recv()
    parent.send((99, "close", None))
    assert parent.poll(3.0), "the worker ignored close even though the device refused to close"
    request, ok, detail = parent.recv()
    assert (request, ok) == (99, False)
    assert "cleanup boom" in detail
    assert thread.is_alive()  # the arm is still held on purpose


# --------------------------------------------------------------------------- #
# follower RPC: a timed-out command must not swallow hold/close
# --------------------------------------------------------------------------- #


def test_follower_rpc_still_forwards_hold_after_a_command_timeout(monkeypatch) -> None:
    from onero_a1_lerobot import sdk_process_leader
    from onero_a1_lerobot.sdk_device_config import SdkArmConfig
    from onero_a1_lerobot.sdk_process_follower import ProcessFollowerDevice

    device = ProcessFollowerDevice(SdkArmConfig(model="A1L", backend="mock"))
    device._pending_command = (1, 1, 0.0)

    def late(_timeout):
        raise TimeoutError("late command")

    device._finish_command_locked = late
    forwarded: list[tuple[str, object]] = []
    monkeypatch.setattr(
        sdk_process_leader.ProcessLeaderDevice,
        "_rpc",
        lambda self, operation, argument=None: forwarded.append((operation, argument)),
    )

    device._rpc("hold")
    assert forwarded == [("hold", None)]

    # A non-hold/close operation must still surface the timeout.
    with pytest.raises(TimeoutError):
        device._rpc("query", 1.0)


# --------------------------------------------------------------------------- #
# session close: the torque-off chain runs even when the hold explodes
# --------------------------------------------------------------------------- #


def test_session_close_disables_arms_when_the_hold_fails() -> None:
    order: list[str] = []

    class Probe(SdkSession):
        def _hold_all(self) -> None:
            raise RuntimeError("hold exploded")

    session = object.__new__(Probe)
    session._closing = threading.Event()
    session.recorder = SimpleNamespace(
        stop=lambda: order.append("recorder.stop"),
        close=lambda: order.append("recorder.close"),
    )
    session.teleop = SimpleNamespace(disconnect=lambda: order.append("teleop.disconnect"))
    session.robot = SimpleNamespace(
        cameras={},
        _disconnect_arms=lambda: order.append("robot.disconnect_arms"),
        disconnect_cameras=lambda: order.append("robot.disconnect_cameras"),
    )
    session._camera_thread = None

    session.close()

    assert order == [
        "recorder.stop",
        "teleop.disconnect",
        "robot.disconnect_arms",
        "robot.disconnect_cameras",
        "recorder.close",
    ]
    assert session.state == "disconnected"


# --------------------------------------------------------------------------- #
# camera: a dead worker must not look like a healthy connection
# --------------------------------------------------------------------------- #


def test_camera_reconnects_after_its_worker_thread_exits() -> None:
    from onero_a1_lerobot.camera import UsbCamera
    from onero_a1_lerobot.camera_config import UsbCameraConfig

    camera = UsbCamera(UsbCameraConfig(backend="mock", device=None, width=64, height=48, fps=30))
    camera.preflight()
    camera.connect()
    try:
        first = None
        for _ in range(100):
            try:
                first = camera.snapshot()
                break
            except Exception:
                time.sleep(0.02)
        assert first is not None, "the mock camera never produced a frame"

        # The worker can exit for any reason; the flag must follow it.
        camera._stop.set()
        camera._thread.join(2.0)
        assert not camera._thread.is_alive()
        assert not camera.is_connected

        # ...so connect() restarts capture instead of reporting a false success.
        camera.connect()
        time.sleep(0.2)
        assert camera._thread.is_alive()
        assert camera.is_connected
        assert camera.snapshot().sequence > first.sequence
    finally:
        camera.disconnect()
