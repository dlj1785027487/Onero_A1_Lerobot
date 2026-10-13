"""SDK freedrive leaders mapped into canonical follower-space actions."""

from __future__ import annotations

import logging
from functools import partial

from .arm_mapping import ArmMapper
from .compat import Teleoperator
from .features import arm_position_keys, gripper_key, ordered_sides
from .sdk_client import SdkDevice, close_devices, hold_devices
from .sdk_config import OneroSdkTeleopConfig
from .startup import run_startup_tasks

logger = logging.getLogger(__name__)


class OneroSdkTeleop(Teleoperator):
    config_class = OneroSdkTeleopConfig
    name = "onero_sdk_teleop"

    def __init__(self, config: OneroSdkTeleopConfig, *, simulation_control_hz: float = 200):
        super().__init__(config)
        config.validate()
        self.config = config
        self.devices: dict[str, SdkDevice] = {}
        self.last_positions: dict[str, list[float]] = {}
        self._simulation_control_hz = simulation_control_hz
        self.mappers = {
            side: ArmMapper(config.mappings[side])
            for side in ordered_sides(config.mappings)
        }
        # Each follower side mirrors the gripper measured on its leader source.
        self._gripper_sources = {
            side: mapper.config.source for side, mapper in self.mappers.items()
            if config.arms[mapper.config.source].gripper_channel
        }

    @property
    def action_features(self) -> dict:
        features: dict[str, type] = {}
        for side in self.mappers:
            features.update(dict.fromkeys(arm_position_keys(side), float))
            if side in self._gripper_sources:
                features[gripper_key(side)] = float
        return features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return bool(self.devices) and all(d.connected for d in self.devices.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _create_devices(self) -> None:
        """Build endpoint owners without enabling any arm."""
        self.config.validate()
        from .sdk_process_leader import ProcessLeaderDevice

        self.devices = {
            name: (
                ProcessLeaderDevice
                if self.config.isolate_can_processes and cfg.backend == "can"
                else SdkDevice
            )(cfg, simulation_control_hz=self._simulation_control_hz)
            for name, cfg in self.config.arms.items()
        }

    def _connection_tasks(self):
        return {
            f"leader {name} ({device.config.channel or device.config.backend})":
                partial(device.connect, leader=True, damping=self.config.damping)
            for name, device in self.devices.items()
        }

    def _supports_parallel_startup(self) -> bool:
        from .sdk_process_leader import ProcessLeaderDevice

        return all(
            device.config.backend != "can" or isinstance(device, ProcessLeaderDevice)
            for device in self.devices.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return
        try:
            self._create_devices()
            for device in self.devices.values():
                device.preflight()
            run_startup_tasks(
                logger, self._connection_tasks(), parallel=self._supports_parallel_startup(),
            )
        except BaseException:
            self.disconnect()
            raise

    def prepare(self, observation: dict[str, float]) -> None:
        if not self.is_connected:
            raise ConnectionError("Teleoperator is not connected")
        states = {
            name: (
                d.query(self.config.stale_state_s)
                if d.config.backend == "mujoco"
                else d.snapshot(self.config.stale_state_s)
            )
            for name, d in self.devices.items()
        }
        for side, mapper in self.mappers.items():
            leader = states[mapper.config.source].joints
            follower = [observation[k] for k in arm_position_keys(side)]
            # Captured for every mapping; only the relative one reads it back.
            mapper.capture(leader, follower)
            if mapper.config.reference == "relative":
                logger.info(
                    "Mapping %s -> %s: mode=%s reference=relative scale=%g; copying the "
                    "leader's movement from leader_ref=%s follower_ref=%s rad",
                    mapper.config.source,
                    side,
                    mapper.config.mode,
                    mapper.config.scale,
                    [round(q, 4) for q in leader],
                    [round(q, 4) for q in follower],
                )
            else:
                logger.info(
                    "Mapping %s -> %s: mode=%s reference=absolute scale=%g offset=%s; every "
                    "joint is driven to the leader's own angle (leader=%s rad), so the "
                    "follower must already be within robot.startup_max_delta_rad of it",
                    mapper.config.source,
                    side,
                    mapper.config.mode,
                    mapper.config.scale,
                    [round(q, 4) for q in mapper.config.offset],
                    [round(q, 4) for q in leader],
                )

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise ConnectionError("Teleoperator is not connected")
        for device in self.devices.values():
            device.step_simulation_leader()
        states = {name: d.snapshot(self.config.stale_state_s) for name, d in self.devices.items()}
        self.last_positions = {name: list(state.joints) for name, state in states.items()}
        action = {}
        for side, mapper in self.mappers.items():
            q = mapper.map(states[mapper.config.source].joints)
            action.update(zip(arm_position_keys(side), map(float, q), strict=True))
            source = self._gripper_sources.get(side)
            if source is not None:
                gripper = states[source].gripper
                action[gripper_key(side)] = float(gripper) if gripper is not None else 0.0
        return action

    def hold(self) -> None:
        hold_devices(self.devices)

    def resume(self) -> None:
        for device in self.devices.values():
            device.resume_leader(self.config.damping)

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        try:
            self.hold()
        except Exception:
            logger.exception("Leader hold during disconnect failed")
        finally:
            close_devices(self.devices)
            self.last_positions.clear()
            for mapper in self.mappers.values():
                mapper.clear()
