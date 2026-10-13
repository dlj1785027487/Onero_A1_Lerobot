"""Pure device and mapping configuration, independent of LeRobot and transports."""

from __future__ import annotations

import math
from dataclasses import dataclass


def positive(value: float, name: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be finite and > 0")


def joint_limit_setting(value, name: str):
    """A scalar applies to all joints; seven values follow J1 through J7."""
    if isinstance(value, (list, tuple)):
        if len(value) != 7:
            raise ValueError(f"{name} must contain seven values for J1 through J7")
        for index, item in enumerate(value, start=1):
            positive(item, f"{name}[J{index}]")
        return tuple(float(item) for item in value)
    positive(value, name)
    return float(value)


def feedforward_scale_setting(value):
    """A scalar or seven J1–J7 factors, each between zero and one."""
    values = value if isinstance(value, (list, tuple)) else (value,)
    if isinstance(value, (list, tuple)) and len(value) != 7:
        raise ValueError("velocity_feedforward_scale must contain seven values for J1 through J7")
    if any(
        isinstance(item, bool) or not isinstance(item, (int, float))
        or not math.isfinite(item) or not 0 <= item <= 1
        for item in values
    ):
        raise ValueError("velocity_feedforward_scale values must be finite numbers between 0 and 1")
    if isinstance(value, (list, tuple)):
        return tuple(float(item) for item in values)
    return float(value)


@dataclass
class SdkArmConfig:
    model: str = "A1R"
    backend: str = "can"
    channel: str | None = None
    mounting: str | None = None
    # CAN attaches the A1 gripper channel and its gravity profile.
    # mock/mujoco use the gravity profile only, without a gripper channel.
    with_gripper: bool = False
    # Visual tool model on Link7, mirroring the SDK's --gripper-model:
    #   "woan_picker" -> the self-developed Onero gripper mechanism (default)
    #   "omnipicker"  -> the legacy OmniPicker mechanism
    #   "none"        -> no tool geometry
    # Selecting a model never changes the CAN protocol, the gravity profile or
    # the TCP; the SDK keeps those deliberately independent.
    gripper_model: str = "woan_picker"
    # Physical CAN gripper protocol, deliberately independent of with_gripper
    # (gravity profile only) and of the visual tool model:
    #   "onero"      -> OneroGripper, DM-compatible MIT protocol (default)
    #   "omnipicker" -> OmniPickerGripper, legacy 8-byte protocol
    #   "none"       -> no gripper CAN frames are ever sent
    # The default preserves the incumbent adapter behavior for existing configs.
    gripper_protocol: str = "onero"
    # Legacy OmniPicker driver options; ignored by the other protocols.
    gripper_can_id: int = 0x08
    gripper_force: int = 0x20
    gripper_velocity: int = 0xFF
    # Passive teach-handle endpoints for a teleoperation leader. The handle
    # reuses the gripper's DM motor and CAN ids but has its own mechanical zero,
    # so the raw position codes measured on the real handle replace the model
    # calibration; otherwise the whole handle travel collapses into a few
    # percent of the gripper range. Codes grow toward closing, like the
    # gripper's own endpoints, so ``open`` is the smaller value. Declaring them
    # is also what marks a leader as a hand-moved teach handle: it is driven
    # read-only instead of being enabled with its arm. Both must be given
    # together, and leaving them out keeps a plain gripper driver on that arm --
    # which is what the single-arm follow configs, where one arm drives another,
    # rely on. Follower arms must leave both unset.
    handle_open_code: int | None = None
    handle_closed_code: int | None = None
    # A gripper -- or teach-handle -- status query is a single CAN round trip that
    # the SDK driver does not retry, and active queries are issued about 50 ms
    # apart. On a slcan/USB link one lost frame used to latch a gripper fault
    # immediately, which then stops the whole session; tolerate this many
    # consecutive missed queries (the default is roughly 150 ms) before reporting
    # the gripper as faulty. Every successful query clears the count, so only a
    # gripper that really stopped answering is reported. Set it to 1 to restore
    # the strict first-miss behaviour.
    gripper_query_failures_before_fault: int = 3
    mock_motion: bool = False
    mock_initial: tuple[float, ...] = (0.0,) * 7
    simulation_initial: tuple[float, ...] | None = None  # Never moves CAN hardware on startup.

    @property
    def gripper_channel(self) -> bool:
        """Whether this arm exposes a controllable gripper action feature."""
        return (
            self.backend == "can"
            and self.with_gripper
            and self.gripper_protocol != "none"
        )

    @property
    def teach_handle(self) -> bool:
        """Whether the gripper channel is a hand-moved teach handle.

        Declaring the measured endpoints is what marks an arm as a teach handle;
        a leader that sets them is driven read-only, and every other arm keeps
        the stock gripper driver. Follower arms cannot reach this state, because
        validation rejects endpoints on them.
        """
        return self.handle_open_code is not None

    def __post_init__(self) -> None:
        if not isinstance(self.with_gripper, bool) or not isinstance(self.mock_motion, bool):
            raise ValueError("with_gripper and mock_motion must be booleans")
        if self.gripper_protocol not in ("none", "omnipicker", "onero"):
            raise ValueError("gripper_protocol must be none, omnipicker or onero")
        if self.gripper_model not in ("none", "omnipicker", "woan_picker"):
            raise ValueError("gripper_model must be none, omnipicker or woan_picker")
        if (isinstance(self.gripper_can_id, bool) or not isinstance(self.gripper_can_id, int)
                or not 0 <= self.gripper_can_id <= 0x7FF):
            raise ValueError("gripper_can_id must be an 11-bit CAN identifier")
        for name in ("gripper_force", "gripper_velocity"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFF:
                raise ValueError(f"{name} must be an integer in [0, 255]")
        for name in ("handle_open_code", "handle_closed_code"):
            code = getattr(self, name)
            if code is not None and (
                isinstance(code, bool) or not isinstance(code, int)
                or not 0 <= code <= 0xFFFF
            ):
                raise ValueError(f"{name} must be a 16-bit DM position code")
        if (self.handle_open_code is None) != (self.handle_closed_code is None):
            raise ValueError("handle_open_code and handle_closed_code must be given together")
        if (isinstance(self.gripper_query_failures_before_fault, bool)
                or not isinstance(self.gripper_query_failures_before_fault, int)
                or self.gripper_query_failures_before_fault < 1):
            raise ValueError("gripper_query_failures_before_fault must be a positive integer")
        if self.handle_open_code is not None and self.handle_open_code == self.handle_closed_code:
            raise ValueError("handle_open_code and handle_closed_code must differ")
        if self.model not in ("A1L", "A1R"):
            raise ValueError("model must be explicit: A1L or A1R")
        if self.backend not in ("can", "mock", "mujoco"):
            raise ValueError("backend must be can, mock or mujoco")
        if self.backend == "can" and (not self.channel or not self.mounting):
            raise ValueError("CAN arms require explicit channel and mounting")
        if self.backend != "can" and self.channel is not None:
            raise ValueError("Simulation arms must not specify a CAN channel")
        if self.mock_motion and self.backend != "mock":
            raise ValueError("mock_motion is only available with backend=mock")
        if len(self.mock_initial) != 7 or not all(math.isfinite(v) for v in self.mock_initial):
            raise ValueError("mock_initial must contain seven finite radians")
        if self.backend != "mock" and any(self.mock_initial):
            raise ValueError("mock_initial is only available with backend=mock")
        if self.simulation_initial is not None and (
            len(self.simulation_initial) != 7
            or not all(math.isfinite(q) for q in self.simulation_initial)
        ):
            raise ValueError("simulation_initial must contain seven finite radians")


@dataclass
class ArmMappingConfig:
    source: str = "left"
    mode: str = "mirror"
    reference: str = "absolute"
    scale: float = 1.0
    offset: tuple[float, ...] = (0.0,) * 7
    joint_signs: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        if self.mode not in ("mirror", "joint"):
            raise ValueError("mapping mode must be mirror or joint")
        if self.mode == "joint":
            if not isinstance(self.joint_signs, (tuple, list)) or len(self.joint_signs) != 7 or any(
                isinstance(sign, bool) or sign not in (-1, 1) for sign in self.joint_signs
            ):
                raise ValueError("joint mode requires seven joint_signs, each -1 or +1")
        elif self.joint_signs is not None:
            raise ValueError("joint_signs requires mode=joint")
        if self.reference not in ("absolute", "relative"):
            raise ValueError("reference must be absolute or relative")
        positive(self.scale, "mapping.scale")
        if len(self.offset) != 7 or not all(math.isfinite(v) for v in self.offset):
            raise ValueError("mapping.offset must contain seven finite radians")
        if self.reference == "relative" and any(self.offset):
            raise ValueError("relative mapping captures its offset; configured offset must be zero")


def validate_channels(*groups: dict[str, SdkArmConfig]) -> None:
    channels: set[str] = set()
    for group in groups:
        for arm in group.values():
            arm.__post_init__()
            if arm.backend == "can":
                if arm.channel in channels:
                    raise ValueError(f"CAN channel is used by more than one arm: {arm.channel}")
                channels.add(arm.channel)
