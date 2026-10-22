"""Shared SDK session for one or two leader/follower routes, with optional recording."""

from __future__ import annotations

import argparse
import json
import logging
import os
import queue
import signal
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from .camera import CameraError
from .camera_config import UsbCameraConfig
from .episode_recording import EpisodeRecorder, RecordingConfig
from .features import arm_position_keys
from .sdk_config import (
    ArmMappingConfig,
    OneroSdkConfig,
    OneroSdkTeleopConfig,
    SdkArmConfig,
    validate_channels,
)
from .sdk_robot import OneroSdk
from .sdk_teleoperator import OneroSdkTeleop
from .startup import run_startup_tasks, startup_stage

logger = logging.getLogger(__name__)

# A teleoperation leader takes its gripper opening from a squeeze handle instead
# of an assembly-dependent CAN gripper. The handle reuses the same DM motor and
# CAN ids as a gripper, so gripper_protocol and gripper_model stay follower-only
# choices and are pinned to these values on the leader; what a leader actually
# needs from its own entry is the measured handle travel, because the handle sits
# at a different mechanical zero than the gripper the model endpoints describe.
_LEADER_GRIPPER_PROTOCOL = "onero"
_LEADER_GRIPPER_MODEL = "woan_picker"


_LEADER_GRIPPER_FIELDS = (
    "gripper_protocol", "gripper_model", "gripper_can_id", "gripper_force", "gripper_velocity",
)


# Camera workers publish at the sensor rate and share the interpreter with the
# 200 Hz control loop and the recorder.  CPython's default 5 ms switch interval
# lets the GIL strand a worker for up to that long, pushing its loop past one
# frame period; a 30 fps recorder then reuses the previous frame and the dataset
# carries duplicated camera samples.  1 ms keeps the wait far inside the 33 ms
# frame budget without measurably slowing the control loop.
_GIL_SWITCH_INTERVAL_S = 0.001


def _leader_gripper_fields(name: str, fields: dict[str, Any]) -> dict[str, Any]:
    """Drop the follower-only gripper selection from a leader arm entry.

    The values are removed before SdkArmConfig validates them, so a leader entry can
    carry anything without failing startup; the pinned defaults are what the leader
    actually uses. ``handle_open_code``/``handle_closed_code`` are deliberately not
    in this list: they are leader-specific measurements and must survive.
    """
    ignored = {key: fields[key] for key in _LEADER_GRIPPER_FIELDS if key in fields}
    if ignored:
        logger.info(
            "Leader %s ignores gripper settings %s; the leader gripper is fixed to %s/%s",
            name, ignored, _LEADER_GRIPPER_PROTOCOL, _LEADER_GRIPPER_MODEL,
        )
    return {key: value for key, value in fields.items() if key not in ignored}


@dataclass
class SdkSessionConfig:
    robot: OneroSdkConfig
    teleop: OneroSdkTeleopConfig
    recording: RecordingConfig = field(default_factory=RecordingConfig)

    def validate(self) -> None:
        self.robot.validate()
        self.teleop.validate()
        self.recording.validate()
        validate_channels(self.robot.arms, self.teleop.arms)
        if set(self.robot.arms) != set(self.teleop.mappings):
            raise ValueError("Every configured follower arm must have exactly one leader mapping")
        # The follower's gripper action is driven by the leader's opening reading,
        # so both sides of a pair must expose the gripper feature together.
        for side, mapping in self.teleop.mappings.items():
            leader = self.teleop.arms[mapping.source]
            follower = self.robot.arms[side]
            if follower.gripper_channel != leader.gripper_channel:
                raise ValueError(
                    f"Follower {side} and leader {mapping.source} disagree on the gripper "
                    f"feature (follower={follower.gripper_channel}, "
                    f"leader={leader.gripper_channel}); the follower gripper action is driven "
                    "by the leader's opening reading, so enable it on both arms or neither"
                )
        if self.robot.bootstrap_relative_follow:
            # The bootstrap turns absolute targets into follower-relative motion so
            # the native LeRobot CLIs can follow without prepare(). A relative
            # mapping already submits a follower-relative target and needs the
            # references prepare() captures, which those CLIs never call, so the
            # two must not be combined on any arm of the session.
            relative = sorted(
                side for side, mapping in self.teleop.mappings.items()
                if mapping.reference != "absolute"
            )
            if relative:
                raise ValueError(
                    "bootstrap_relative_follow requires reference='absolute' on every mapping; "
                    f"{', '.join(relative)} is not absolute"
                )


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate configuration key: {key}")
        result[key] = value
    return result


def _preferred_control_cpus(allowed: set[int]) -> set[int] | None:
    """Select faster cores only when the OS exposes complete capacity data."""
    capacities = {}
    try:
        for cpu in allowed:
            capacity = int(Path(f"/sys/devices/system/cpu/cpu{cpu}/cpu_capacity").read_text())
            if capacity <= 0:
                return None
            capacities[cpu] = capacity
    except (OSError, ValueError):
        return None
    if len(set(capacities.values())) < 2:
        return None
    largest = max(capacities.values())
    return {cpu for cpu, capacity in capacities.items() if capacity == largest}


def _tune_gil_switch_interval() -> None:
    """Cap how long a camera worker can be stranded waiting for the GIL.

    Must run before the camera workers are created.  A user or caller that
    already asked for a shorter interval keeps it.
    """
    current = sys.getswitchinterval()
    if current <= _GIL_SWITCH_INTERVAL_S:
        return
    sys.setswitchinterval(_GIL_SWITCH_INTERVAL_S)
    logger.info(
        "GIL switch interval %.4fs -> %.4fs for camera workers sharing the "
        "interpreter with the control loop", current, _GIL_SWITCH_INTERVAL_S,
    )


def _apply_control_cpu_affinity(config: SdkSessionConfig) -> set[int] | None:
    requested = config.robot.control_cpu_affinity
    if requested is None:
        return None
    if not any(arm.backend == "can" for group in (config.robot.arms, config.teleop.arms)
               for arm in group.values()):
        logger.info("Control CPU affinity skipped for simulation")
        return None
    if not callable(getattr(os, "sched_getaffinity", None)) or not callable(
        getattr(os, "sched_setaffinity", None)
    ):
        if requested == "auto":
            logger.info("Control CPU affinity auto: OS selection retained (affinity API unavailable)")
            return None
        raise RuntimeError("control_cpu_affinity requires OS CPU affinity support")
    try:
        previous = os.sched_getaffinity(0)
    except OSError:
        if requested == "auto":
            logger.info("Control CPU affinity auto: OS selection retained (allowed CPUs unavailable)")
            return None
        raise
    if requested == "auto":
        mask = _preferred_control_cpus(previous)
        if mask is None:
            logger.info("Control CPU affinity auto: OS selection retained (no distinct CPU capacities)")
            return None
    else:
        mask = set(requested)
    if not mask <= previous:
        raise ValueError(
            f"control_cpu_affinity={sorted(mask)} is outside the allowed CPUs {sorted(previous)}"
        )
    try:
        os.sched_setaffinity(0, mask)
    except OSError:
        if requested == "auto":
            logger.info("Control CPU affinity auto: OS selection retained (affinity request rejected)")
            return None
        raise
    try:
        actual = os.sched_getaffinity(0)
        if actual != mask:
            raise RuntimeError(f"CPU affinity applied as {sorted(actual)}, requested {sorted(mask)}")
        logger.info("Control CPU affinity=%s; spawned arm processes inherit this mask", sorted(actual))
    except BaseException as exc:
        os.sched_setaffinity(0, previous)
        if requested == "auto" and isinstance(exc, (OSError, RuntimeError)):
            logger.info("Control CPU affinity auto: previous selection restored (verification failed)")
            return None
        raise
    return previous


def load_session_config(path: str | Path, *, simulate: bool | str = False) -> SdkSessionConfig:
    backend = "mock" if simulate is True else simulate
    if backend not in (False, None, "mock", "mujoco"):
        raise ValueError("Simulation backend must be mock or mujoco")
    data = json.loads(Path(path).expanduser().read_text(), object_pairs_hook=_unique_object)
    if set(data) - {"robot", "teleop", "recording"}:
        raise ValueError("Unknown session configuration fields")
    robot_data, teleop_data = dict(data["robot"]), dict(data["teleop"])
    robot_data["arms"] = {name: SdkArmConfig(**cfg) for name, cfg in robot_data["arms"].items()}
    robot_data["cameras"] = {
        name: UsbCameraConfig(**cfg) for name, cfg in robot_data.get("cameras", {}).items()
    }
    teleop_data["arms"] = {
        name: SdkArmConfig(**_leader_gripper_fields(name, cfg))
        for name, cfg in teleop_data["arms"].items()
    }
    teleop_data["mappings"] = {
        name: ArmMappingConfig(**cfg) for name, cfg in teleop_data["mappings"].items()
    }
    if backend:
        # Synthetic images have no V4L2 controls, so drop the exposure settings
        # instead of keeping values that UsbCameraConfig rejects on a mock camera.
        robot_data["cameras"] = {
            name: replace(
                cfg, backend="mock", device=None, auto_exposure=None, exposure_absolute=None,
            )
            for name, cfg in robot_data["cameras"].items()
        }
        for group, leader in ((robot_data["arms"], False), (teleop_data["arms"], True)):
            for name, cfg in group.items():
                kwargs = dict(
                    backend=backend, channel=None, mock_motion=leader and backend == "mock"
                )
                if backend == "mujoco":
                    # Retain mounting metadata for the combined viewer.
                    # SdkDevice keeps MuJoCo's dynamics in its native base frame.
                    kwargs.update(with_gripper=False, mock_initial=(0.0,) * 7)
                elif cfg.simulation_initial is not None:
                    kwargs["mock_initial"] = cfg.simulation_initial
                group[name] = replace(cfg, **kwargs)
    config = SdkSessionConfig(
        OneroSdkConfig(**robot_data),
        OneroSdkTeleopConfig(**teleop_data),
        RecordingConfig(**data.get("recording", {})),
    )
    config.validate()
    return config


class SdkSession:
    def __init__(self, config: SdkSessionConfig, *, recorder_factory=EpisodeRecorder):
        config.validate()
        self.config = config
        self.robot = OneroSdk(config.robot, on_fault=self.enter_fault)
        self.teleop = OneroSdkTeleop(config.teleop, simulation_control_hz=config.robot.control_hz)
        if self.robot.action_features != self.teleop.action_features:
            raise ValueError("Robot and teleoperator feature schemas differ")
        self.recorder = recorder_factory(
            config.recording, self.robot.observation_features, self.robot.action_features
        )
        self.state = "disconnected"
        self.frames = 0
        self.last_action: dict[str, float] = {}
        self.last_requested_action: dict[str, float] = {}
        self.last_observation: dict[str, Any] = {}
        self._simulation_rebase = False
        self.stop_reason: str | None = None
        self._hold_started = False
        self._hold_failures: dict[str, str] = {}
        self._next_stop_warning = 0.0
        self._closing = threading.Event()
        self._camera_done = threading.Event()
        self._camera_error: str | None = None
        self._camera_software_checked = False
        self._camera_software_error: str | None = None
        self._camera_thread: threading.Thread | None = None
        self._background_cpus: tuple[int, ...] | None = None

    def configure_background_cpus(self, cpus: tuple[int, ...] | None) -> None:
        self._background_cpus = cpus
        self.recorder.configure_cpu_affinity(cpus)

    def _prepare_camera_software(self) -> None:
        """Finish OpenCV's sys.path-changing bootstrap before any spawn."""
        if self._camera_software_checked:
            return
        try:
            if self.robot.cameras:
                with startup_stage(logger, "camera software dependencies (no USB I/O)"):
                    for camera in self.robot.cameras.values():
                        camera.preflight()
        except Exception as exc:
            self._camera_software_error = str(exc)
            logger.error("Camera software unavailable; CAN startup can continue: %s", exc)
        # A failed import is not retried by background camera tasks or r;
        # fix dependencies and restart, rather than mutating spawn's path.
        self._camera_software_checked = True

    def _start_cameras(self) -> None:
        if self._closing.is_set():
            return
        if self._camera_thread is not None and self._camera_thread.is_alive():
            return
        if self._camera_software_error is not None:
            self._camera_error = self._camera_software_error
            self._camera_done.set()
            logger.error("Camera software is unavailable; restart after fixing dependencies")
            self.recorder.fail(f"camera software unavailable: {self._camera_software_error}")
            return
        self._camera_error = None
        self._camera_done.clear()
        if not self.robot.cameras:
            self._camera_done.set()
            return

        def initialize() -> None:
            try:
                if self._background_cpus and hasattr(os, "sched_setaffinity"):
                    # Linux pid=0 changes only this thread. The camera workers
                    # it creates inherit this mask, not the control CPU mask.
                    os.sched_setaffinity(0, self._background_cpus)
                    logger.info("Camera background CPU affinity=%s", list(self._background_cpus))
                with startup_stage(logger, "camera background initialization"):
                    self.robot.connect_cameras(cancelled=self._closing.is_set)
            except BaseException as exc:
                self._camera_error = str(exc)
                if not self._closing.is_set():
                    logger.error("Camera initialization failed; CAN following is independent: %s", exc)
                    self.recorder.fail(f"camera initialization failed: {exc}")
            finally:
                if self._closing.is_set():
                    self.robot.disconnect_cameras()
                self._camera_done.set()

        self._camera_thread = threading.Thread(
            target=initialize, name="a1_camera_startup", daemon=True,
        )
        try:
            self._camera_thread.start()
        except BaseException as exc:
            self._camera_thread = None
            self._camera_error = str(exc)
            self._camera_done.set()
            logger.error("Could not start camera initialization: %s", exc)

    def set_simulation_auto(self, source: str, enabled: bool) -> None:
        if self.state != "following" or any(
            d.config.backend != "mujoco"
            for d in (*self.robot.devices.values(), *self.teleop.devices.values())
        ):
            raise RuntimeError("Simulation input changes require a following MuJoCo session")
        device = self.teleop.devices[source]
        if device.simulation_auto != enabled:
            device.set_simulation_auto(enabled)
            # Defer I/O until the next control tick, away from input callbacks.
            self._simulation_rebase = True

    def connect(self) -> None:
        if self.state != "disconnected":
            raise RuntimeError("Session already connected")
        if self._camera_thread is not None and self._camera_thread.is_alive():
            raise RuntimeError("Previous camera initialization is still stopping")
        started = time.monotonic()
        self._closing.clear()
        try:
            with startup_stage(logger, "all arm dependency checks"):
                self.robot._create_devices()
                self.teleop._create_devices()
                for device in (*self.robot.devices.values(), *self.teleop.devices.values()):
                    device.preflight()
            # Only import shared software here. Device initialization and
            # dataset initialization below remain independent and concurrent.
            self._prepare_camera_software()
            # Independent services start before the arm connection barrier.
            # No LeRobot dataset imports or dataset I/O run on this thread.
            self._start_cameras()
            if self.config.recording.enabled or self.config.recording.repo_id or self.config.recording.task:
                try:
                    self.recorder.prepare()
                    if self.config.recording.enabled:
                        self.recorder.start()
                except Exception as exc:
                    self.recorder.fail_service(str(exc))
            with startup_stage(logger, "all arm connections"):
                if self.robot._supports_parallel_startup() and self.teleop._supports_parallel_startup():
                    logger.info("Starting %d arm endpoints in parallel", len(self.robot.devices) + len(self.teleop.devices))
                    run_startup_tasks(logger, {
                        **self.teleop._connection_tasks(), **self.robot._connection_tasks(),
                    })
                else:
                    # Non-isolated CAN followers cannot continuously refresh
                    # their hold while waiting for another arm to initialize.
                    logger.info("Using ordered arm startup for non-isolated CAN endpoints")
                    run_startup_tasks(logger, self.teleop._connection_tasks(), parallel=False)
                    run_startup_tasks(logger, self.robot._connection_tasks(), parallel=False)
            # Process followers stay in background hold until every endpoint
            # has finished startup. Only now refresh feedback and clear holds.
            with startup_stage(logger, "feedback and mapping references"):
                self.robot.prepare()
                self.teleop.prepare(self.robot.get_joint_observation())
            self._simulation_rebase = False
            self.stop_reason = None
            self._hold_started = False
            self._hold_failures.clear()
            self.state = "following"
            logger.info(
                "Arm startup complete in %.3fs; cameras=%s; recording=%s",
                time.monotonic() - started,
                ("error" if self._camera_error else "ready") if self._camera_done.is_set() else "starting",
                self.recorder.status,
            )
        except BaseException:
            self.close()
            raise

    def tick(self) -> None:
        if self.state in ("paused", "fault"):
            if not self._hold_all() and self.state != "fault":
                self.enter_fault(RuntimeError("Position hold failed; manual investigation required"))
            self._log_stopped()
            return
        if self.state != "following":
            raise RuntimeError(f"Cannot follow in session state {self.state}")
        try:
            # Prefer a known SDK stop over a secondary stale-state or limit
            # symptom from another endpoint. This adds no transport queries.
            self._check_sdk_faults()
            if self._simulation_rebase:
                self.robot.hold()
                self.robot.prepare()
                self.teleop.prepare(self.robot.get_joint_observation())
                self._simulation_rebase = False
                logger.info("Simulation input changed; captured new following references")
            action = self.teleop.get_action()
            observation = self.robot.get_joint_observation()
            submitted = self.robot.send_action(action)
            self.last_observation, self.last_action = observation, submitted
            self.last_requested_action = action
            self.frames += 1
            timestamp = time.monotonic()
            if self.recorder.wants_frame(timestamp):
                if not self._camera_done.is_set():
                    return  # Controls were submitted; camera initialization is independent.
                if self._camera_error is not None:
                    self.recorder.fail(f"Camera initialization failed: {self._camera_error}")
                    return
                try:
                    recording_observation = {
                        **observation,
                        **self.robot.get_camera_observation(include_timestamps=True, for_recording=True),
                    }
                except CameraError as exc:
                    self.recorder.fail(f"Camera unavailable: {exc}")
                else:
                    self.last_observation = recording_observation
                    self.recorder.submit(recording_observation, submitted, timestamp)
        except Exception as exc:
            self.enter_fault(exc)
            raise

    def _check_sdk_faults(self) -> None:
        for role, devices in (("leader", self.teleop.devices), ("follower", self.robot.devices)):
            for name, device in devices.items():
                if device.fault_reason is not None:
                    raise RuntimeError(f"{role}/{name} SDK fault: {device.fault_reason}")
                # A latched gripper fault is independent of the arm fault. The
                # isolated CAN workers fold it into fault_reason, but a directly
                # connected device reports it only here, so check it as well.
                gripper_fault = getattr(device, "gripper_fault", None)
                if gripper_fault is not None:
                    raise RuntimeError(f"{role}/{name} gripper fault: {gripper_fault}")

    def enter_fault(self, error: Exception) -> None:
        """Latch a fault without disabling healthy arms or clearing SDK stops."""
        if self.state != "fault":
            self.state = "fault"
            self.stop_reason = str(error)
            if self.recorder.status in ("starting", "recording"):
                self.recorder.fail(f"control fault: {error}")
            else:
                self.recorder.stop()
            self._hold_started = False
            self._next_stop_warning = 0.0
            logger.error("Following fault: %s", error)
        self._hold_all()
        self._log_stopped()

    def _hold_devices(self):
        groups = [("leader", self.teleop.devices)]
        if self.config.robot.send_action:
            groups.append(("follower", self.robot.devices))
        return {
            f"{role}/{name} ({device.config.model}, {device.config.channel or device.config.backend})": device
            for role, devices in groups for name, device in devices.items()
            if device.connected
        }

    def _hold_all(self) -> bool:
        devices = self._hold_devices()
        self.robot.last_command_velocities.clear()
        pending = {name: d for name, d in devices.items() if name not in self._hold_failures}

        def hold(item):
            name, device = item
            try:
                if device.hold() is False:
                    return name, device.fault_reason or "SDK cannot hold this arm"
            except Exception as exc:
                return name, device.fault_reason or str(exc)
            return name, None

        # SDK stop() can take multiple control periods. Stop separate CAN
        # endpoints together; one decelerating arm must not delay the others.
        if not self._hold_started and len(pending) > 1:
            with ThreadPoolExecutor(max_workers=len(pending), thread_name_prefix="onero_hold") as pool:
                results = list(pool.map(hold, pending.items()))
        else:
            results = [hold(item) for item in pending.items()]
        self._hold_started = True
        for name, error in results:
            if error is not None:
                self._hold_failures[name] = error
                logger.critical(
                    "POSITION HOLD UNAVAILABLE for %s: %s. Support this arm; "
                    "no automatic re-enable. Other controllable arms remain in hold.", name, error,
                )
        # The SDK may finish disabling motors and publish its error callback
        # after the first hold attempt. Report that root cause when it arrives.
        for name in self._hold_failures:
            reason = devices[name].fault_reason if name in devices else None
            if reason and reason != self._hold_failures[name]:
                self._hold_failures[name] = reason
                logger.critical("SDK fault confirmed for %s: %s", name, reason)
        return not self._hold_failures

    def _log_stopped(self) -> None:
        if self.state != "fault" or time.monotonic() < self._next_stop_warning:
            return
        held = [name for name in self._hold_devices() if name not in self._hold_failures]
        instruction = "Fault is latched; p cannot resume. Investigate before restarting."
        logger.warning(
            "Session %s; following and recording stopped; reason=%s; "
            "holding=%s; HOLD UNAVAILABLE=%s. %s Support all arms before q/Ctrl+C (disables motors).",
            self.state, self.stop_reason, held, self._hold_failures,
            instruction,
        )
        self._next_stop_warning = time.monotonic() + 5.0

    def log_diagnostics(self) -> None:
        """Report the last completed control frame without extra CAN queries."""
        if self.state != "following" or not self.last_action:
            return
        for side, mapping in self.config.teleop.mappings.items():
            keys = arm_position_keys(side)
            target = [self.last_requested_action[key] for key in keys]
            limits = self.robot.devices[side].limits
            report = {
                "frame": self.frames,
                "source": mapping.source,
                "follower": side,
                "mode": mapping.mode,
                "send_action": self.config.robot.send_action,
                "leader": [round(q, 4) for q in self.teleop.last_positions[mapping.source]],
                "mapped_target": [round(q, 4) for q in target],
                "sdk_target": [round(self.last_action[key], 4) for key in keys],
                "feedback": [round(self.last_observation[key], 4) for key in keys],
                "continuous_follow": self.config.robot.continuous_follow,
                "clip_joint_limits": self.config.robot.clip_joint_limits,
                "velocity_feedforward": [
                    round(v, 4) for v in self.robot.last_command_velocities.get(side, [0.0] * 7)
                ],
                "clipped_joints": [
                    i + 1 for i, (q, (lo, hi)) in enumerate(zip(target, limits, strict=True))
                    if self.config.robot.clip_joint_limits and (q < lo or q > hi)
                ],
                "out_of_bounds_joints": [
                    i + 1 for i, (q, (lo, hi)) in enumerate(zip(target, limits, strict=True))
                    if q < lo or q > hi
                ],
            }
            gripper = self.robot._gripper_key_by_side.get(side)
            if gripper is not None:
                report["gripper_target"] = round(self.last_action.get(gripper, float("nan")), 3)
                report["gripper_feedback"] = round(
                    self.last_observation.get(gripper, float("nan")), 3
                )
            logger.info("Follow diagnostics %s", json.dumps(report))

    def pause(self) -> None:
        if self.state != "following":
            raise RuntimeError("Can only pause a following session")
        self.recorder.stop()
        self.state = "paused"
        self._hold_started = False
        if not self._hold_all():
            self.enter_fault(RuntimeError("Position hold failed while pausing"))

    def resume(self) -> None:
        if self.state != "paused":
            raise RuntimeError("Only a paused session can resume; faults require reconnecting")
        try:
            self._check_sdk_faults()
            self.robot.prepare()
            self.teleop.resume()
            self.teleop.prepare(self.robot.get_joint_observation())
            self._simulation_rebase = False
            self._hold_started = False
            self.stop_reason = None
            self.state = "following"
        except Exception as exc:
            self.enter_fault(exc)
            raise

    def start_recording(self) -> None:
        if self.state != "following":
            raise RuntimeError("Recording can only start while following")
        if self._camera_done.is_set() and self._camera_error is not None:
            self._start_cameras()
        self.recorder.start()

    def stop_recording(self) -> None:
        self.recorder.stop()

    def close(self) -> None:
        self._closing.set()
        self.recorder.stop()
        # Hold is best-effort and already reports its own failures; it must not
        # be allowed to skip the torque-off chain below, which is what actually
        # releases the arms.
        try:
            self._hold_all()
        except BaseException:
            logger.exception("Position hold failed during shutdown; disabling anyway")
        # Disable and detach every arm before potentially slow dataset finalization.
        try:
            self.teleop.disconnect()
        finally:
            try:
                self.robot._disconnect_arms()
            finally:
                self.state = "disconnected"
                try:
                    # connect() polls _closing; wait for its owner to finish
                    # before final camera cleanup, avoiding connect/close races.
                    if self._camera_thread is not None:
                        timeout = 10.0 + sum(
                            camera.config.connect_timeout_s for camera in self.robot.cameras.values()
                        )
                        self._camera_thread.join(timeout)
                    if self._camera_thread is not None and self._camera_thread.is_alive():
                        logger.error("Camera initialization has not exited; its owner will finish cleanup")
                    else:
                        self._camera_thread = None
                        self.robot.disconnect_cameras()
                finally:
                    self.recorder.close()


def build_session_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CAN SDK leader/follower session (one or two arms)"
    )
    parser.add_argument("--session-config", required=True)
    parser.add_argument(
        "--simulate",
        nargs="?",
        choices=("mock", "mujoco"),
        const="mock",
        default=False,
        help="Replace ALL endpoints: mock (default) or the SDK's MuJoCo physics backend",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open a combined SDK MuJoCo window (use --simulate mujoco)",
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument(
        "--diagnostics-interval", type=float, default=None, metavar="SECONDS",
        help="Log leader, mapped/limited targets and follower feedback every SECONDS (no recording)",
    )
    parser.add_argument(
        "--reference",
        choices=("absolute", "relative"),
        default=None,
        help="Override teleop.reference for this run; omit to keep the session config",
    )
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument("--record", dest="record", action="store_true", default=None)
    recording.add_argument("--no-record", dest="record", action="store_false")
    parser.add_argument("--repo-id")
    parser.add_argument("--task")
    parser.add_argument("--root", help="Exact LeRobot dataset directory")
    return parser


def run_session_cli(argv: list[str], *, record: bool = False) -> None:
    args = build_session_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", force=True
    )
    config = load_session_config(args.session_config, simulate=args.simulate)
    if args.reference is not None:
        # Applied before validate() so the session-level value is propagated to
        # every mapping exactly like the JSON field.
        config.teleop.reference = args.reference
    if args.duration is not None or args.diagnostics_interval is not None:
        from .sdk_config import positive

        if args.duration is not None:
            positive(args.duration, "duration")
        if args.diagnostics_interval is not None:
            positive(args.diagnostics_interval, "diagnostics_interval")
    if args.record is not None:
        config.recording.enabled = args.record
    elif record:
        config.recording.enabled = True
    for name in ("repo_id", "task", "root"):
        if getattr(args, name) is not None:
            setattr(config.recording, name, getattr(args, name))
    config.validate()
    if args.viewer:
        from .sdk_viewer import SdkSimulationViewer

        SdkSimulationViewer.preflight(config)
    session = SdkSession(config)
    viewer = None
    timing = None
    commands: queue.Queue[str] = queue.Queue()

    def read_commands() -> None:
        for line in sys.stdin:
            commands.put(line.strip().lower())

    shutdown_signal = None
    previous_handlers = {}

    def request_shutdown(signum, _frame) -> None:
        nonlocal shutdown_signal
        # Do not interrupt CAN I/O or cleanup mid-call. Further signals leave
        # cleanup running so every configured arm can receive its disable.
        shutdown_signal = signum

    if threading.current_thread() is threading.main_thread():
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_shutdown)

    started = 0.0
    previous_affinity = None
    try:
        previous_affinity = _apply_control_cpu_affinity(config)
        background_cpus = (
            tuple(sorted(previous_affinity - os.sched_getaffinity(0)))
            if previous_affinity is not None else ()
        )
        session.configure_background_cpus(background_cpus or None)
        logger.info(
            "Camera/recording background CPUs=%s; control CPUs=%s",
            list(background_cpus) if background_cpus else "shared (no separate CPUs available)",
            sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else "OS default",
        )
        _tune_gil_switch_interval()
        session.connect()
        if args.viewer and shutdown_signal is None:
            viewer = SdkSimulationViewer(session)
            viewer.open()
            # Window creation can take seconds; refresh before starting control.
            session.robot.prepare()
            session.teleop.prepare(session.robot.get_joint_observation())
        if args.diagnostics_interval is not None:
            from .sdk_timing import FollowTiming

            timing = FollowTiming(session)
        started = time.monotonic()
        logger.info(
            "Following %s at requested %.1f Hz; recording=%s; send_action=%s; "
            "continuous_follow=%s; clip_joint_limits=%s",
            ", ".join(config.robot.arms),
            config.robot.control_hz,
            session.recorder.status,
            config.robot.send_action,
            config.robot.continuous_follow,
            config.robot.clip_joint_limits,
        )
        logger.info(
            "Effective follower limits (J1..J7): velocity=%s rad/s; acceleration=%s rad/s^2; max_step=%s rad",
            {side: limiter.joint_max_velocity.tolist() for side, limiter in session.robot._limiters.items()},
            {side: limiter.joint_max_acceleration.tolist() for side, limiter in session.robot._limiters.items()},
            {side: limiter.max_step for side, limiter in session.robot._limiters.items()},
        )
        logger.info("Velocity feedforward scale (J1..J7)=%s", config.robot.velocity_feedforward_scale)
        logger.info("Commands + Enter: r=record on/off, p=pause/resume following, q=quit")
        logger.info("Exit disables all connected arms; pause keeps position control active")
        logger.info(
            "Runtime control faults keep the service alive "
            "and hold controllable arms. SDK emergency stops are never automatically cleared."
        )
        if viewer:
            logger.info(
                "Viewer keys: 1-7 select joint; -/= tap/hold jog; A auto/manual; [/] select leader; P/R/Q session commands"
            )
        if sys.stdin.isatty():
            threading.Thread(target=read_commands, name="onero_console", daemon=True).start()
        period = 1 / config.robot.control_hz
        next_diagnostics = started
        while shutdown_signal is None and (
            args.duration is None or time.monotonic() - started < args.duration
        ):
            tick_start = time.monotonic()
            if viewer:
                if not viewer.is_running:
                    break
                for command in viewer.poll_commands():
                    commands.put(command)
            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command == "q":
                break
            if command == "r":
                try:
                    if session.recorder.status in ("off", "error"):
                        session.start_recording()
                    else:
                        session.stop_recording()
                    logger.info("Recording: %s", session.recorder.status)
                except Exception as exc:
                    logger.error("Recording request rejected: %s", exc)
            if command == "p":
                try:
                    if session.state == "paused":
                        session.resume()
                    else:
                        session.pause()
                    logger.info("Session: %s", session.state)
                except Exception as exc:
                    logger.error("Pause/resume request rejected: %s", exc)
            try:
                session.tick()
            except Exception as exc:
                # Keep refreshing healthy holds instead of entering the
                # shutdown finally block, which intentionally disables arms.
                session.enter_fault(exc)
            if args.diagnostics_interval is not None and tick_start >= next_diagnostics:
                session.log_diagnostics()
                next_diagnostics = tick_start + args.diagnostics_interval
            if viewer:
                viewer.sync()
            if timing is not None:
                timing.maybe_log()
            delay = period - (time.monotonic() - tick_start)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        logger.info("Stopping session")
    finally:
        try:
            if shutdown_signal is not None:
                logger.info("Stopping session after %s", signal.Signals(shutdown_signal).name)
            try:
                try:
                    if timing is not None:
                        timing.close()
                finally:
                    session.close()
            finally:
                if viewer:
                    viewer.close()
        finally:
            try:
                if previous_affinity is not None:
                    os.sched_setaffinity(0, previous_affinity)
                    logger.info("Restored CPU affinity=%s", sorted(previous_affinity))
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)
        elapsed = time.monotonic() - started if started else 0.0
        logger.info(
            "Session finished: control_frames=%d elapsed=%.2fs recording_frames=%d recording_error=%s",
            session.frames,
            elapsed,
            session.recorder.frames,
            session.recorder.error,
        )
    if session.recorder.error:
        raise SystemExit("Session ended with a recording error: " + session.recorder.error)
