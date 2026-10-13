"""LeRobot configurations and re-exports of the pure device configuration."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from .camera_config import UsbCameraConfig
from .compat import RobotConfig, TeleoperatorConfig
from .features import ordered_sides
from .sdk_device_config import (
    ArmMappingConfig,
    SdkArmConfig,
    feedforward_scale_setting,
    joint_limit_setting,
    positive,
    validate_channels,
)


@dataclass(kw_only=True)
class OneroSdkConfig(RobotConfig):
    arms: dict[str, SdkArmConfig] = field(default_factory=dict)
    cameras: dict[str, UsbCameraConfig] = field(default_factory=dict)
    send_action: bool = False
    control_hz: float = 100.0
    max_velocity: float | tuple[float, ...] = 0.3
    max_acceleration: float | tuple[float, ...] = 1.0
    velocity_feedforward_scale: float | tuple[float, ...] = 1.0
    max_control_gap_s: float = 0.25
    stale_state_s: float = 0.25
    startup_max_delta_rad: float = 0.15
    max_tracking_error_rad: float = 0.5
    continuous_follow: bool = False
    clip_joint_limits: bool = False
    isolate_can_processes: bool = False
    control_cpu_affinity: tuple[int, ...] | str | None = None
    # The native LeRobot CLIs (lerobot-teleoperate / lerobot-record) build the
    # robot and teleoperator themselves and never call OneroSdkTeleop.prepare(),
    # so a relative mapping has no captured reference and cannot run there.
    # With this switch on, the first action submitted after prepare() is
    # reinterpreted as the follower's current position, and every later action
    # is applied as an offset from it (q0 + (target - first_target)) -- exactly
    # what a relative mapping produces from its captured pair of references.
    # The mapping itself must therefore be reference="absolute" with a zero
    # offset, because the native path cannot capture references at all. No
    # alignment motion is needed: the first command is the measured position.
    # Off by default, because a policy rollout or a dataset replay submits
    # absolute targets that must not be shifted.
    bootstrap_relative_follow: bool = False

    def validate(self) -> None:
        camera_devices: set[str] = set()
        for name, camera in self.cameras.items():
            if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
                raise ValueError("Camera names must start with a letter and contain letters, digits or underscores")
            if not isinstance(camera, UsbCameraConfig):
                raise TypeError(f"Camera {name} must be a UsbCameraConfig")
            camera.validate()
            if camera.backend == "usb":
                device = str(Path(camera.device).resolve())
                if device in camera_devices:
                    raise ValueError(f"USB camera device is configured more than once: {camera.device}")
                camera_devices.add(device)
        if not isinstance(self.send_action, bool):
            raise ValueError("send_action must be a boolean, not a quoted string")
        if not isinstance(self.continuous_follow, bool):
            raise ValueError("continuous_follow must be a boolean")
        if not isinstance(self.clip_joint_limits, bool):
            raise ValueError("clip_joint_limits must be a boolean")
        if not isinstance(self.isolate_can_processes, bool):
            raise ValueError("isolate_can_processes must be a boolean")
        if not isinstance(self.bootstrap_relative_follow, bool):
            raise ValueError("bootstrap_relative_follow must be a boolean")
        self.max_velocity = joint_limit_setting(self.max_velocity, "max_velocity")
        self.max_acceleration = joint_limit_setting(self.max_acceleration, "max_acceleration")
        self.velocity_feedforward_scale = feedforward_scale_setting(self.velocity_feedforward_scale)
        affinity = self.control_cpu_affinity
        if affinity is not None:
            if isinstance(affinity, str):
                if affinity != "auto":
                    raise ValueError("control_cpu_affinity string must be 'auto'")
            elif (not isinstance(affinity, (list, tuple)) or not affinity
                    or any(isinstance(cpu, bool) or not isinstance(cpu, int) or cpu < 0
                           for cpu in affinity)
                    or len(set(affinity)) != len(affinity)):
                raise ValueError("control_cpu_affinity must be 'auto' or a nonempty list of unique nonnegative CPU integers")
            else:
                self.control_cpu_affinity = tuple(affinity)
        ordered_sides(self.arms)
        validate_channels(self.arms)
        for name, arm in self.arms.items():
            if arm.handle_open_code is not None:
                raise ValueError(
                    f"Follower arm {name} must not define teach-handle endpoints; "
                    "handle_open_code/handle_closed_code describe the teleoperation "
                    "leader handle, while a follower uses the gripper's own calibration"
                )
        for name in (
            "control_hz",
            "max_control_gap_s",
            "stale_state_s",
            "startup_max_delta_rad",
            "max_tracking_error_rad",
        ):
            positive(getattr(self, name), name)
        if 1 / self.control_hz >= self.max_control_gap_s:
            raise ValueError("max_control_gap_s must exceed one control period")


@dataclass(kw_only=True)
class OneroSdkTeleopConfig(TeleoperatorConfig):
    arms: dict[str, SdkArmConfig] = field(default_factory=dict)
    mappings: dict[str, ArmMappingConfig] = field(default_factory=dict)
    damping: float = 0.3
    stale_state_s: float = 0.25
    isolate_can_processes: bool = False

    def validate(self) -> None:
        if not isinstance(self.isolate_can_processes, bool):
            raise ValueError("isolate_can_processes must be a boolean")
        if not self.arms:
            raise ValueError("Configure at least one leader arm")
        ordered_sides(self.mappings)
        validate_channels(self.arms)
        sources = []
        for mapping in self.mappings.values():
            mapping.__post_init__()
            if mapping.source not in self.arms:
                raise ValueError(f"Unknown leader source: {mapping.source}")
            sources.append(mapping.source)
        if len(set(sources)) != len(sources) or set(sources) != set(self.arms):
            raise ValueError("Each leader must have exactly one follower mapping")
        if not math.isfinite(self.damping) or self.damping < 0:
            raise ValueError("damping must be finite and >= 0")
        positive(self.stale_state_s, "stale_state_s")


if hasattr(RobotConfig, "register_subclass"):
    OneroSdkConfig = RobotConfig.register_subclass("onero_sdk")(OneroSdkConfig)
    OneroSdkTeleopConfig = TeleoperatorConfig.register_subclass("onero_sdk_teleop")(
        OneroSdkTeleopConfig
    )
