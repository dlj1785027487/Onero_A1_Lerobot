"""``SdkDevice.gripper_command`` 的故障路径测试：不连接 CAN、不启动 SDK。

真机现象：SDK 的 ``OneroGripper`` 用每一帧反馈的 status 半字节重算 ``_enabled``
（``armsdk/onero_sdk/grippers.py`` 的 ``_apply_feedback``），所以一帧异常反馈就能让
一个健康的电机被判成"未使能"，随后的一条位置帧被拒绝，在适配器里升级成 following
fault。这里用替身把该协议行为固定下来，避免以后改动把它退化回去。
"""

from __future__ import annotations

import pytest

from onero_a1_lerobot.sdk_client import SdkDevice
from onero_a1_lerobot.sdk_device_config import SdkArmConfig


class FakeOneroGripper:
    """Replicates the SDK driver's enable/status contract without a bus."""

    def __init__(self) -> None:
        self._enabled = True
        self.enable_calls = 0
        self.positions: list[float] = []
        self.enable_error: Exception | None = None

    def enable(self) -> None:
        self.enable_calls += 1
        if self.enable_error is not None:
            raise self.enable_error
        self._enabled = True

    def set_position(self, position: float) -> None:
        if not self._enabled:
            raise RuntimeError("Onero gripper is not enabled")
        self.positions.append(position)

    def drop_enable_from_feedback(self) -> None:
        """What a single bad feedback frame does to the SDK's derived flag."""
        self._enabled = False


def make_device(*, leader: bool = False) -> tuple[SdkDevice, FakeOneroGripper]:
    device = SdkDevice(
        SdkArmConfig(
            model="A1R",
            backend="can",
            channel="can1",
            mounting="humanoid_right",
            with_gripper=True,
            gripper_protocol="onero",
        )
    )
    device._leader = leader
    gripper = FakeOneroGripper()
    device._gripper = gripper
    return device, gripper


def test_healthy_gripper_is_commanded_without_reenabling() -> None:
    device, gripper = make_device()
    device.gripper_command(0.25)
    assert gripper.positions == [0.25]
    assert gripper.enable_calls == 0


def test_disabled_from_feedback_is_reenabled_and_retried() -> None:
    device, gripper = make_device()
    gripper.drop_enable_from_feedback()
    device.gripper_command(0.5)
    assert gripper.enable_calls == 1
    assert gripper.positions == [0.5]


def test_retry_failure_still_propagates() -> None:
    """A gripper that cannot be re-enabled must still stop the session."""
    device, gripper = make_device()
    gripper.drop_enable_from_feedback()
    gripper.enable_error = RuntimeError("Onero gripper did not respond to enable")
    with pytest.raises(RuntimeError, match="did not respond to enable"):
        device.gripper_command(0.5)
    assert gripper.positions == []


def test_leader_gripper_is_never_reenabled() -> None:
    """A read-only teach handle must never have enable re-asserted."""
    device, gripper = make_device(leader=True)
    gripper.drop_enable_from_feedback()
    with pytest.raises(RuntimeError, match="is not enabled"):
        device.gripper_command(0.5)
    assert gripper.enable_calls == 0


def test_latched_gripper_fault_still_blocks_commands() -> None:
    device, gripper = make_device()
    device._gripper_fault = "Gripper fault: protocol=onero code=9 state=9"
    with pytest.raises(RuntimeError, match="Gripper fault"):
        device.gripper_command(0.5)
    assert gripper.positions == []


def test_out_of_range_position_is_rejected_before_any_io() -> None:
    device, gripper = make_device()
    with pytest.raises(ValueError):
        device.gripper_command(1.5)
    assert gripper.positions == []


def test_sdk_derives_enabled_from_the_feedback_status_nibble() -> None:
    """Pin the SDK contract the retry above depends on.

    ``OneroGripper._apply_feedback`` recomputes ``_enabled`` from every feedback
    frame, and a status of 0 is reported as ``fault_code == 0`` -- i.e. as a
    healthy reading -- so the adapter's own gripper-fault latch cannot see it.
    If a future SDK changes either half of this, the retry needs re-review.
    """
    onero_sdk = pytest.importorskip("onero_sdk")
    gripper = onero_sdk.OneroGripper(model="A1R")

    # status nibble 1 = enabled
    gripper._apply_feedback(bytes([0x10, 0x20, 0x00, 0x00, 0x00, 0x00]))
    assert gripper._enabled is True
    assert gripper.state.fault_code == 0

    # status nibble 0 = motor reports itself disabled: no fault, but commands die
    gripper._apply_feedback(bytes([0x00, 0x20, 0x00, 0x00, 0x00, 0x00]))
    assert gripper._enabled is False
    assert gripper.state.motion_state == 0
    assert gripper.state.fault_code == 0
    with pytest.raises(RuntimeError, match="is not enabled"):
        gripper.set_position(0.5)
