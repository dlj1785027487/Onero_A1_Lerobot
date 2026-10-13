"""Single/dual arm LeRobot Robot using the SDK's direct CAN interface."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from functools import partial
from typing import Any

import numpy as np

from .arm_mapping import JointSlewLimiter, joint_vector
from .camera import UsbCamera
from .compat import Robot
from .features import arm_position_keys, gripper_key, ordered_sides
from .recording_image import RecordingImage
from .sdk_client import SdkDevice, close_devices, hold_devices
from .sdk_config import OneroSdkConfig
from .sdk_process_follower import ProcessFollowerDevice
from .startup import run_startup_tasks
from .utils import normalize_action_dict

logger = logging.getLogger(__name__)

# Gripper transactions block on CAN round trips, so the 100+ Hz control loop
# resends only on meaningful change and never more often than ~20 Hz.
_GRIPPER_DELTA = 0.02
_GRIPPER_INTERVAL_S = 0.05


class OneroSdk(Robot):
    config_class = OneroSdkConfig
    name = "onero_sdk"

    def __init__(self, config: OneroSdkConfig, *, on_fault=None):
        super().__init__(config)
        config.validate()
        self.config = config
        self._on_fault = on_fault
        self.cameras = {name: UsbCamera(camera) for name, camera in config.cameras.items()}
        self.devices: dict[str, SdkDevice | ProcessFollowerDevice] = {}
        self._limiters: dict[str, JointSlewLimiter] = {}
        self._bootstrap_refs: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._last_send: float | None = None
        self._first_send = True
        self._fault: str | None = None
        self._next_limit_warning: dict[str, float] = {}
        self.last_command_velocities: dict[str, list[float]] = {}
        self._command_frame = 0
        self._last_gripper: dict[str, tuple[float | None, float]] = {}
        self._cache_features()

    def _cache_features(self) -> None:
        self._keys_by_side = {
            side: arm_position_keys(side) for side in ordered_sides(self.config.arms)
        }
        self._gripper_key_by_side = {
            side: gripper_key(side) for side in self._keys_by_side
            if self.config.arms[side].gripper_channel
        }
        keys: list[str] = []
        for side in self._keys_by_side:
            keys.extend(self._keys_by_side[side])
            if side in self._gripper_key_by_side:
                keys.append(self._gripper_key_by_side[side])
        self._feature_keys = tuple(keys)
        self._feature_set = frozenset(self._feature_keys)
        self._feature_schema = dict.fromkeys(self._feature_keys, float)

    @property
    def action_features(self) -> dict:
        # Callers may extend the returned schema without changing our cache.
        return self._feature_schema.copy()

    @property
    def observation_features(self) -> dict:
        return {
            **self.action_features,
            **{name: camera.shape for name, camera in self.config.cameras.items()},
        }

    def connect_cameras(self, *, cancelled: Callable[[], bool] | None = None) -> None:
        """Wait for valid camera frames before enabling any arm."""
        try:
            for camera in self.cameras.values():
                camera.preflight()
            run_startup_tasks(logger, {
                f"camera {name} first frame": partial(camera.connect, cancelled=cancelled)
                for name, camera in self.cameras.items() if not camera.is_connected
            })
            if cancelled is not None and cancelled():
                raise RuntimeError("Camera startup cancelled")
            # An earlier camera may have failed while a later one warmed up.
            # Recheck every cached frame before any arm is enabled.
            for camera in self.cameras.values():
                camera.snapshot()
        except BaseException:
            self.disconnect_cameras()
            raise

    def disconnect_cameras(self) -> None:
        for name, camera in self.cameras.items():
            try:
                camera.disconnect()
            except Exception:
                logger.exception("Could not disconnect USB camera %s", name)

    @property
    def is_connected(self) -> bool:
        return bool(self.devices) and all(d.connected for d in self.devices.values())

    @property
    def is_calibrated(self) -> bool:
        return True  # SDK owns zero offsets; this adapter never rewrites them.

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _create_devices(self) -> None:
        """Build endpoint owners without enabling any arm."""
        self.config.validate()
        self._cache_features()
        self._command_frame = 0
        self.devices = {
            side: (
                ProcessFollowerDevice(
                    self.config.arms[side],
                    simulation_control_hz=self.config.control_hz,
                    max_control_gap_s=self.config.max_control_gap_s,
                )
                if self.config.isolate_can_processes and self.config.arms[side].backend == "can"
                else SdkDevice(self.config.arms[side], simulation_control_hz=self.config.control_hz)
            )
            for side in self._keys_by_side
        }

    def _connect_device(self, device: SdkDevice | ProcessFollowerDevice) -> None:
        device.connect()
        if isinstance(device, ProcessFollowerDevice) and device.hold() is not True:
            raise RuntimeError(f"Follower startup hold unavailable: {device.config.channel}")

    def _connection_tasks(self):
        return {
            f"follower {side} ({device.config.channel or device.config.backend})":
                partial(self._connect_device, device)
            for side, device in self.devices.items()
        }

    def _supports_parallel_startup(self) -> bool:
        # Direct CAN endpoints have no background hold loop during startup.
        return all(
            device.config.backend != "can" or isinstance(device, ProcessFollowerDevice)
            for device in self.devices.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return
        try:
            self._create_devices()
            for device in self.devices.values():
                device.preflight()
            self.connect_cameras()
            run_startup_tasks(
                logger, self._connection_tasks(), parallel=self._supports_parallel_startup(),
            )
            self.prepare()
        except BaseException:
            self.disconnect()
            raise

    def prepare(self) -> None:
        if not self.is_connected:
            raise ConnectionError("Robot is not connected")
        if self._fault is not None:
            raise RuntimeError(
                "Robot fault is latched; disconnect and investigate before reconnecting"
            )
        self._limiters.clear()
        self._next_limit_warning.clear()
        self.last_command_velocities.clear()
        for side, device in self.devices.items():
            # Session boundaries may follow slow connection/recording setup.
            # Refresh once here; active control uses cached servo feedback.
            q = device.query(self.config.stale_state_s).joints
            bounds = np.asarray(device.limits)
            if self.config.clip_joint_limits and (
                np.any(np.asarray(q) < bounds[:, 0]) or np.any(np.asarray(q) > bounds[:, 1])
            ):
                raise RuntimeError(f"{side} feedback is outside the SDK model limits")
            max_velocity = self.config.max_velocity
            if self.config.continuous_follow:
                model_velocities = getattr(
                    device, "joint_velocity_limits", (device.max_joint_velocity,) * 7,
                )
                # Each joint retains its own SDK model ceiling; a slower
                # shoulder joint need not impose that ceiling on the wrist.
                max_velocity = np.minimum(max_velocity, model_velocities).tolist()
            self._limiters[side] = JointSlewLimiter(
                q, device.limits, max_velocity,
                self.config.max_acceleration,
                continuous=self.config.continuous_follow,
                max_step=device.servo_max_step if self.config.continuous_follow else None,
                clip_joint_limits=self.config.clip_joint_limits,
            )
        # Keep all process followers refreshing hold while other feedback
        # queries finish; release only after every limiter is initialized.
        for device in self.devices.values():
            device.clear_hold()
        # Every session boundary -- connect, resume, simulation rebase --
        # re-captures the bootstrap pair from the position the follower really
        # holds then, exactly as a relative mapping re-captures its references.
        self._bootstrap_refs.clear()
        self._last_send = None
        self._first_send = True
        self._last_gripper.clear()

    def _bootstrap_follower_target(
        self, side: str, target: np.ndarray, measured: np.ndarray,
    ) -> np.ndarray:
        """Reinterpret the first absolute target as this side's own reference.

        The native LeRobot CLIs never call ``OneroSdkTeleop.prepare()``, so a
        relative mapping cannot run there. Latching the measured position and
        the first target once turns an absolute mapping into the same
        ``q0 + (target - first_target)`` motion a relative mapping produces,
        without touching the teleoperator or the CLI. Only joint positions go
        through this; the gripper target is already an absolute opening.
        """
        reference = self._bootstrap_refs.get(side)
        if reference is None:
            q0 = measured.copy()
            self._bootstrap_refs[side] = (q0, target.copy())
            logger.info(
                "Bootstrap relative follow for %s: follower q0=%s rad; "
                "first target=%s rad; later targets are applied as offsets from these",
                side,
                [round(q, 4) for q in q0.tolist()],
                [round(q, 4) for q in target.tolist()],
            )
            return q0
        q0, first_target = reference
        return q0 + (target - first_target)

    def get_joint_observation(self) -> dict[str, float]:
        """Read arm feedback independently of camera availability."""
        if not self.is_connected:
            raise ConnectionError("Robot is not connected")
        obs = {}
        for side, device in self.devices.items():
            state = (
                device.snapshot(self.config.stale_state_s)
                if self.config.send_action
                else device.query(self.config.stale_state_s)
            )
            q = state.joints
            obs.update(zip(self._keys_by_side[side], q, strict=True))
            if side in self._gripper_key_by_side:
                gripper = getattr(state, "gripper", None)
                obs[self._gripper_key_by_side[side]] = (
                    float(gripper) if gripper is not None else 0.0
                )
        return obs

    def get_camera_observation(
        self, *, include_timestamps: bool = False, for_recording: bool = False,
    ) -> dict[str, Any]:
        """Read cached RGB for policies, or original JPEG bytes for recording."""
        observation = {}
        for name, camera in self.cameras.items():
            frame = camera.snapshot()
            if for_recording and frame.jpeg is not None:
                observation[name] = RecordingImage(frame.jpeg, camera.config.shape)
            else:
                observation[name] = frame.image
            if include_timestamps:
                observation[f"camera.{name}.time_s"] = frame.timestamp
                observation[f"camera.{name}.sequence"] = float(frame.sequence)
        return observation

    def get_observation(self) -> dict[str, Any]:
        return {**self.get_joint_observation(), **self.get_camera_observation()}

    def send_action(self, action: dict) -> dict[str, float]:
        if not self.is_connected:
            raise ConnectionError("Robot is not connected")
        if self._fault is not None:
            raise RuntimeError(f"Robot fault is latched: {self._fault}")
        now = time.monotonic()
        dt = 1 / self.config.control_hz if self._last_send is None else now - self._last_send
        try:
            normalized = normalize_action_dict(action, self._feature_keys)
            if normalized.keys() != self._feature_set:
                raise ValueError("Action must contain exactly the configured follower features")
            if dt > self.config.max_control_gap_s:
                raise RuntimeError(
                    "Control stream interrupted; refusing to catch up to old targets"
                )
            observation = self.get_joint_observation()
            targets = {}
            for side in self.devices:
                keys = self._keys_by_side[side]
                target = joint_vector([normalized[k] for k in keys])
                measured = joint_vector([observation[k] for k in keys])
                if self.config.bootstrap_relative_follow:
                    # Before the start guard and the limiter, so the first
                    # command is the measured position rather than a jump.
                    target = self._bootstrap_follower_target(side, target, measured)
                expected = self._limiters[side].position
                if (
                    not self._first_send
                    and not self.config.clip_joint_limits
                    and self.devices[side].config.backend != "mock"
                ):
                    # The real SDK independently clips position commands. Use
                    # its bounded reference for the error check only; do not
                    # change the submitted target or the planner's state.
                    # The limiter already copied fixed CAN model bounds in
                    # prepare(); mock/simulation limits remain mutable.
                    bounds = (
                        self._limiters[side].limits
                        if self.devices[side].config.backend == "can"
                        else np.asarray(self.devices[side].limits)
                    )
                    expected = np.clip(expected, bounds[:, 0], bounds[:, 1])
                # send_action=False only suppresses the outgoing command: the
                # follower is deliberately held while the leader keeps moving, so
                # distance guards must not fire (they used to latch a tracking
                # error after the leader moved max_tracking_error_rad away).
                if self.config.send_action and self._first_send:
                    first_delta = np.abs(target - measured)
                    if np.max(first_delta) > self.config.startup_max_delta_rad:
                        over = np.flatnonzero(first_delta > self.config.startup_max_delta_rad)
                        details = "; ".join(
                            f"J{i + 1} target={target[i]:+.4f} follower={measured[i]:+.4f} "
                            f"delta={target[i] - measured[i]:+.4f}"
                            for i in over
                        )
                        raise RuntimeError(
                            f"{side} ({self.config.arms[side].model}) first target is too far "
                            f"from the follower: {details} rad, limit "
                            f"{self.config.startup_max_delta_rad:.4f} rad. Full-range (absolute) "
                            "mapping commands the leader's own pose, so pose the follower within "
                            "the limit before starting, use incremental (relative) mapping, or "
                            "raise startup_max_delta_rad to let the limiter slew to the leader"
                        )
                if (
                    self.config.send_action
                    and not self._first_send
                    and np.max(abs(expected - measured))
                    > self.config.max_tracking_error_rad
                ):
                    raise RuntimeError(f"{side} follower tracking error exceeds configured limit")
                targets[side] = target
            commands = {}
            for side, q in targets.items():
                if (
                    self.config.clip_joint_limits
                    and now >= self._next_limit_warning.get(side, -float("inf"))
                ):
                    bounds = self._limiters[side].limits
                    clipped = np.flatnonzero((q < bounds[:, 0]) | (q > bounds[:, 1]))
                    if len(clipped):
                        details = "; ".join(
                            f"J{i + 1} target={q[i]:.4f} -> "
                            f"{np.clip(q[i], *bounds[i]):.4f} "
                            f"range=[{bounds[i, 0]:.4f}, {bounds[i, 1]:.4f}] rad"
                            for i in clipped
                        )
                        logger.warning("%s (%s) target clipped: %s", side, self.config.arms[side].model, details)
                        self._next_limit_warning[side] = now + 5.0
                try:
                    commands[side] = self._limiters[side].step(q, max(dt, 1e-6))
                except RuntimeError as exc:
                    raise RuntimeError(
                        f"{side} ({self.config.arms[side].model}) motion limiter: {exc}"
                    ) from exc
            command_velocities = {
                side: (
                    self._limiters[side].velocity * self.config.velocity_feedforward_scale
                ).tolist()
                if self.config.continuous_follow else [0.0] * 7
                for side in commands
            }
            if self.config.send_action:
                # All arms have passed validation before any command is sent.
                self._send_commands(commands, command_velocities)
                self._send_gripper_commands(normalized)
            self._first_send = False
            self._last_send = now
            self.last_command_velocities = command_velocities
            return {
                **{
                    key: float(q[i])
                    for side, q in commands.items()
                    for i, key in enumerate(self._keys_by_side[side])
                },
                **{key: float(normalized[key]) for key in self._gripper_key_by_side.values()},
            }
        except Exception as exc:
            self._fault = str(exc)
            try:
                if self._on_fault is None:
                    self.hold()
                else:
                    self._on_fault(exc)
            except Exception:
                logger.exception("Follower hold after a fault failed")
            raise

    def _send_commands(
        self, commands: dict[str, np.ndarray], command_velocities: dict[str, list[float]],
    ) -> None:
        """Overlap independent CAN transactions, with at most one frame in flight."""
        self._command_frame += 1
        error: BaseException | None = None
        try:
            for side, q in commands.items():
                device = self.devices[side]
                if isinstance(device, ProcessFollowerDevice):
                    device.begin_command(
                        q,
                        velocities=(
                            command_velocities[side]
                            if self.config.continuous_follow else None
                        ),
                        frame_id=self._command_frame,
                    )
            for side, q in commands.items():
                device = self.devices[side]
                if not isinstance(device, ProcessFollowerDevice):
                    if self.config.continuous_follow:
                        device.command(q, velocities=command_velocities[side])
                    else:
                        device.command(q)
        except BaseException as exc:
            error = exc
        finally:
            # Even a failed submission or one failed arm must not leave the
            # other arm's transaction pending when fault handling requests hold.
            for side in commands:
                device = self.devices[side]
                if isinstance(device, ProcessFollowerDevice) and device.command_pending:
                    try:
                        device.finish_command()
                    except BaseException as exc:
                        if error is None:
                            error = exc
                        else:
                            logger.error("%s follower transaction also failed: %s", side, exc)
        if error is not None:
            raise error

    def _send_gripper_commands(self, normalized: dict[str, float]) -> None:
        """Forward gripper targets, rate-limited because transactions block."""
        now = time.monotonic()
        for side, key in self._gripper_key_by_side.items():
            target = normalized[key]
            last_value, last_time = self._last_gripper.get(side, (None, 0.0))
            if (
                last_value is not None
                and abs(target - last_value) <= _GRIPPER_DELTA
                and now - last_time < _GRIPPER_INTERVAL_S
            ):
                continue
            try:
                self.devices[side].gripper_command(target)
            except Exception as exc:
                raise RuntimeError(
                    f"{side} ({self.config.arms[side].model}) gripper command failed: {exc}"
                ) from exc
            self._last_gripper[side] = (target, now)

    def hold(self) -> None:
        self.last_command_velocities.clear()
        if self.config.send_action:
            hold_devices(self.devices)

    def _disconnect_arms(self) -> None:
        try:
            self.hold()
        except Exception:
            logger.exception("Follower hold during disconnect failed")
        finally:
            close_devices(self.devices)
            self._limiters.clear()
            self._bootstrap_refs.clear()
            self._next_limit_warning.clear()
            self._last_send = None
            self._fault = None

    def disconnect(self) -> None:
        try:
            self._disconnect_arms()
        finally:
            self.disconnect_cameras()


OneroSdkRobot = OneroSdk
