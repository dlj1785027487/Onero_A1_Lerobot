"""Arm-level SDK access. CAN is direct; this module never imports ROS2."""

from __future__ import annotations

import logging
import math
import threading
import time
from inspect import signature
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .arm_mapping import joint_vector
from .sdk_device_config import SdkArmConfig, positive
from .startup import startup_stage

logger = logging.getLogger(__name__)

# Gripper transactions block up to the SDK feedback timeout (50 ms), so active
# state queries are rate-limited. A commanded follower stays fresh from its
# own set_position feedback instead.
_GRIPPER_FRESH_S = 0.1
_GRIPPER_QUERY_S = 0.05

# A teleoperation handle is left enabled with zero gains so that its drive stops
# braking the shaft. If a motor still falls back to the braked state, stop
# re-declaring the target after this many tries rather than flooding the bus on
# every status query.
_MAX_HANDLE_RELEASES = 5


def _is_slcan_channel(channel: str | None) -> bool:
    return isinstance(channel, str) and channel.startswith("/dev/")


# The SDK verifies all seven motor states before it enables anything, with one
# un-retried feedback query per motor and a 50 ms deadline. A single lost CAN
# frame therefore aborts the whole connect -- measured on slcan/CANable links as
# a several-percent event while the machine is busy starting up, and observed to
# hit the same joint on two consecutive attempts when the link is marginal. Four
# attempts at 50 ms apart still cost under 0.5 s of startup on a healthy link.
# Nothing has been enabled during this check, so a retry is safe and cannot leave
# a half-enabled arm.
_CONNECT_ATTEMPTS = 4
_CONNECT_RETRY_DELAY_S = 0.05
_PRE_ENABLE_STATE_HINT = "before enable"


def _retryable_connect_failure(exc: BaseException) -> bool:
    """Whether a failed connect may be retried.

    Only the pre-enable motor state check qualifies: it runs before any enable
    frame, so every motor is still disabled.
    """
    return _PRE_ENABLE_STATE_HINT in str(exc)


def _joint_velocities(values) -> list[float] | None:
    if values is None:
        return None
    result = np.asarray(values, dtype=float)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError("Expected seven finite joint velocities in radians/second")
    return result.tolist()


def _log_can_queue(channel: str) -> None:
    """Read interface metadata without opening a socket or changing the link."""
    try:
        length = int(Path(f"/sys/class/net/{channel}/tx_queue_len").read_text(encoding="ascii"))
    except (OSError, ValueError) as exc:
        logger.debug("Cannot read CAN %s TX queue length: %s", channel, exc)
        return
    logger.info("CAN %s TX queue length=%d", channel, length)
    if length < 32:
        logger.warning(
            "CAN %s TX queue is small for seven-joint command bursts; "
            "configure it with: sudo bash scripts/setup_can.sh --tx-queue %s",
            channel, channel,
        )


def _build_gripper(config: SdkArmConfig, *, handle_calibration: bool = False):
    """Instantiate the explicitly configured physical gripper driver.

    ``Arm`` binds the driver to its own CAN transport and enables/disables it
    together with the arm; a failed gripper enable therefore fails connect. The
    protocol is selected by ``gripper_protocol`` and stays independent of the
    gravity profile (``with_gripper``) and of the visual tool model.

    ``handle_calibration`` switches to the teach-handle path: the handle reuses
    the gripper motor but is moved by hand and sits at a different mechanical
    zero, so it is driven read-only -- never enabled, never commanded -- and its
    measured endpoints replace the model calibration, which would otherwise
    compress the whole handle travel into a few percent of the gripper range.
    The caller enables this only for a leader that declares both endpoints, so
    an ordinary gripper arm is never affected.
    """
    protocol = config.gripper_protocol
    if protocol == "omnipicker":
        from onero_sdk import OmniPickerGripper

        return OmniPickerGripper(
            can_id=config.gripper_can_id,
            force=config.gripper_force,
            velocity=config.gripper_velocity,
        )
    if protocol == "onero":
        from onero_sdk import OneroGripper

        if handle_calibration:
            return _passive_handle_gripper(OneroGripper, config)
        # Measured left/right endpoints follow the explicit A1L/A1R model.
        return OneroGripper(model=config.model)
    return None


def _passive_handle_gripper(base, config: SdkArmConfig):
    """Build a read-only Onero driver for the leader's hand-moved handle.

    ``_build_gripper`` only reaches this path for a leader that declares both
    ``handle_open_code`` and ``handle_closed_code``. ``Arm`` enables and disables
    its end effector together with the arm, and a commanded follower keeps
    setting positions, so both are overridden here. An enabled handle fights the
    operator: a DM motor holds its shaft stiffly while it is enabled and keeps
    that state across sessions, so ``enable()`` releases the motor instead of
    torqueing it on, and a position frame would drag it; only the passive status
    query stays active, which is all the leader needs to publish
    ``{side}_arm.gripper.pos``.
    """

    class PassiveHandleGripper(base):
        # ``Arm.__init__`` re-applies the model calibration to every bound CAN
        # end effector, which would silently discard measured handle endpoints.
        # The flag is set once they have been applied below; until then the base
        # constructor's own call must still go through.
        _measured_handle_codes = False

        def configure_for_model(self, model: str) -> None:
            if self._measured_handle_codes:
                logger.debug(
                    "Keeping measured handle endpoints on %s (%s) instead of the %s "
                    "model calibration the arm would re-apply",
                    config.channel, config.model, model,
                )
                return
            super().configure_for_model(model)

        # The running motor is worth reporting once, and a handle that keeps
        # dropping back to the braked state must not flood the bus.
        _status_logged = False
        _release_retries = 0
        _release_note = "not released"

        def _mit_payload(self, kp: int, kd: int, code: int) -> bytes:
            """Pack a DM MIT frame with zero feedforward and the given gains."""
            v = self._encode_uint(0.0, -self.max_velocity, self.max_velocity, 12)
            tau = self._encode_uint(0.0, -self.max_torque, self.max_torque, 12)
            return bytes([
                (code >> 8) & 0xFF, code & 0xFF,
                (v >> 4) & 0xFF,
                ((v & 0xF) << 4) | ((kp >> 8) & 0xF), kp & 0xFF,
                (kd >> 4) & 0xFF,
                ((kd & 0xF) << 4) | ((tau >> 8) & 0xF), tau & 0xFF,
            ])

        def _send_then_read(self, payload: bytes, *, lead: bytes | None = None) -> int | None:
            """Send an optional lead frame and ``payload`` back to back, then read status.

            Both frames go out under one I/O lock with no read in between, so a
            motor that ignores control frames while it is disabled is never left
            holding a position between the enable frame and the zero-gain target
            that follows it.
            """
            try:
                bus = self._require_bus()
                with self._io_lock:
                    if lead is not None:
                        bus.send(self.command_id, lead)
                    bus.send(self.command_id, payload)
                    frame = bus.recv_until(
                        self.feedback_id, timeout=self._feedback_timeout
                    )
                if frame is not None:
                    self._apply_feedback(frame.data)
                    return self.state.motion_state
            except Exception:
                logger.warning(
                    "Could not reach the teleoperation handle on %s; it may stay "
                    "stiff until the motor is power-cycled",
                    config.channel, exc_info=True,
                )
            finally:
                # Only a zero-gain target is ever declared here, never a
                # position, so the handle is not "enabled" in the sense
                # set_position() checks.
                self._enabled = False
            return None

        def release(self) -> int | None:
            """Leave the handle motor turning freely.

            A DM drive that is powered but uncommanded holds its phases shorted,
            and that brakes the shaft hard: a handle on such a drive is stiff the
            moment the robot is switched on and loose again once the power is
            off. A disable frame does nothing for it -- these handles do not even
            answer one -- so the released state is reached the other way round,
            by leaving the motor enabled with its target rewritten to
            kp=kd=torque=0. The drive then actively cancels the back-EMF current
            instead of letting it circulate, and the shaft turns with no
            perceptible resistance.

            A disabled motor ignores control frames instead of waking up on
            them, which is why the enable frame is sent immediately before the
            target: sending the target alone leaves the motor braked, which is
            exactly the state being escaped here.

            The position term carries no gain, so the operator's own hand still
            sets the position -- nothing here ever drives the handle, and
            ``set_position()`` keeps refusing commands. Returns the status the
            motor reports afterwards (1 is the free zero-gain state, 0 means it
            stayed braked, None means no reply), and leaves a note in
            ``_release_note`` for the caller to log.
            """
            code = self._encode_uint(
                self.state.raw_position, self.min_position, self.max_position, 16
            )
            status = self._send_then_read(self._mit_payload(0, 0, code), lead=self._ENABLE)
            if status is None:
                self._release_note = "no reply to the zero-gain target"
            else:
                self._release_note = f"zero-gain target set (motor reports status={status})"
            return status

        def enable(self) -> None:
            # ``Arm.enable()`` reaches every bound end effector. For a hand-moved
            # handle the torque-on state it asks for is exactly what must be
            # avoided, so the call releases the motor instead of enabling it.
            self.release()
            logger.info("Teleoperation handle on %s: %s", config.channel, self._release_note)

        def disable(self) -> None:
            self.release()

        def refresh(self):
            state = super().refresh()
            if not self._status_logged:
                self._status_logged = True
                logger.info(
                    "Teleoperation handle on %s reports motor status=%s while running",
                    config.channel, state.motion_state,
                )
            if state.motion_state == 0 and self._release_retries < _MAX_HANDLE_RELEASES:
                # The drive fell back to the powered-but-uncommanded state that
                # brakes the shaft, so the handle is stiff again. Declare the
                # zero-gain target once more, and give up after a few tries so a
                # motor that simply refuses cannot flood the bus.
                self._release_retries += 1
                self.release()
                state = super().refresh()
                logger.warning(
                    "Teleoperation handle on %s reported the braked state (status=0); "
                    "re-released it (status now=%s, attempt %d/%d)",
                    config.channel, state.motion_state,
                    self._release_retries, _MAX_HANDLE_RELEASES,
                )
            return state

        def set_position(self, position: float) -> None:
            raise RuntimeError(
                "The teleoperation handle is read-only; no gripper command is sent to a leader arm"
            )

    gripper = PassiveHandleGripper(model=config.model)
    if config.handle_open_code is None or config.handle_closed_code is None:
        # A guard, not a supported configuration: _build_gripper only takes this
        # path for a leader that declares both endpoints.
        raise ValueError(
            "A read-only teach handle requires both handle_open_code and handle_closed_code"
        )
    # The SDK only exposes per-model endpoint presets, so a measured handle needs
    # these two attributes replaced. Keep the assignment here, in one place, so
    # the dependency stays visible.
    gripper._open_code = int(config.handle_open_code)
    gripper._closed_code = int(config.handle_closed_code)
    # Arm construction and every later reconfiguration must now leave the
    # measured travel alone.
    gripper._measured_handle_codes = True
    return gripper


def _mujoco_backend(config: SdkArmConfig, control_hz: float):
    from onero_sdk.mujoco_backend import MujocoBackend

    if config.simulation_initial is None:
        return MujocoBackend(model_name=config.model, control_dt=1 / control_hz)

    class InitializedBackend(MujocoBackend):
        def enable(self):
            super().enable()
            # Initialize before Arm.enable captures the safety/reference state.
            return self.reset(list(config.simulation_initial))

    return InitializedBackend(model_name=config.model, control_dt=1 / control_hz)


class _MockGripper:
    """In-memory gripper for ``backend=mock``.

    Mirrors the slice of the SDK end-effector interface that ``SdkDevice`` drives
    -- a passive status query, a normalized opening in [0, 1], enable/disable --
    so a simulated session publishes and accepts exactly the features its config
    declares, the gripper channel included. A follower's measured opening follows
    its last command, which exercises the whole gripper path; a leader stays
    read-only, like the hand-moved handle it stands in for, and with
    ``mock_motion`` its opening sweeps on its own so the mirroring path runs
    without an operator touching anything.
    """

    def __init__(self, config: SdkArmConfig, *, leader: bool, opening: float = 1.0) -> None:
        self.config = config
        self._leader = leader
        self._read_only = leader
        self._opening = float(opening)
        self._enabled = False
        self._started = time.monotonic()
        self.state = SimpleNamespace(
            timestamp=0.0, fault_code=None, motion_state=1, opening=self._opening,
        )
        self._publish()

    def _publish(self) -> None:
        self.state.timestamp = time.monotonic()
        self.state.motion_state = 1
        self.state.fault_code = None
        self.state.opening = self._opening

    def enable(self) -> None:
        self._enabled = True
        self._publish()

    def disable(self) -> None:
        self._enabled = False
        self._publish()

    def refresh(self):
        """The passive status query; a simulated leader's hand moves slowly."""
        if self._leader and self.config.mock_motion:
            phase = time.monotonic() - self._started
            # Sweep off both endpoints so neither extreme is parked on.
            self._opening = 0.575 + 0.425 * math.sin(0.7 * phase)
        self._publish()
        return self.state

    def set_position(self, position: float) -> None:
        if self._read_only:
            raise RuntimeError(
                "A simulated leader gripper is read-only; no gripper command is sent "
                "to a leader arm"
            )
        if not self._enabled:
            raise RuntimeError("This simulated gripper is not enabled")
        self._opening = float(position)
        self._publish()

    def get_position(self) -> float:
        return float(self._opening)


class SdkDevice:
    def __init__(self, config: SdkArmConfig, *, simulation_control_hz: float = 200):
        self.config = config
        positive(simulation_control_hz, "simulation_control_hz")
        self._simulation_control_hz = simulation_control_hz
        self.arm = None
        self._gripper = None
        self._gripper_query = 0.0
        self._gripper_commanded = False
        self._gripper_silent = False
        self._gripper_fault: str | None = None
        self._gripper_query_failures = 0
        self._fault = threading.Event()
        self._fault_detail = ""
        self._fault_hold_reported = False
        self._state = None
        self._leader = False
        self._started = 0.0
        self._source_stamp: float | None = None
        self._host_update = 0.0
        self._held = None
        # Mock limits are deliberately a synthetic envelope, not hardware limits.
        self.limits = [(-math.pi, math.pi)] * 7
        self.servo_max_step: float | None = None
        self.joint_velocity_limits: tuple[float, ...] = (math.inf,) * 7
        self.max_joint_velocity = math.inf
        self._model = None
        self._simulation_backend = None
        self._simulation_target = np.zeros(7)
        self._simulation_anchor = np.zeros(7)
        self._simulation_auto = True

    def preflight(self) -> None:
        self.config.__post_init__()
        self.servo_max_step = None
        self.joint_velocity_limits = (math.inf,) * 7
        self.max_joint_velocity = math.inf
        if self.config.backend == "mock":
            return
        try:
            from onero_sdk.model_registry import get_model
            from onero_sdk.safety import SafetyConfig
        except ImportError as exc:
            extra = "sim" if self.config.backend == "mujoco" else "dynamics"
            raise RuntimeError(
                f"Install the SDK: python3 -m pip install 'onero-sdk[{extra}]>=0.3,<0.4' "
                "(or point A1_ARM_SDK_PATH at a local source tree during setup)"
            ) from exc
        self._model = get_model(self.config.model)
        if self._model.num_joints != 7:
            raise ValueError("This adapter requires seven-joint arms")
        self.limits = [(j.p_min, j.p_max) for j in self._model.joints]
        # Arm is constructed without a custom safety config below. Mirror its
        # public default limits so planned position and velocity can agree
        # before the SDK applies its final, independent safety clamping.
        safety = SafetyConfig()
        self.servo_max_step = safety.max_delta_per_step
        self.joint_velocity_limits = tuple(
            min(safety.max_velocity, joint.v_max) for joint in self._model.joints
        )
        self.max_joint_velocity = min(self.joint_velocity_limits)
        if self.config.backend == "mujoco":
            try:
                import mujoco  # noqa: F401
            except ImportError as exc:
                raise RuntimeError("MuJoCo simulation requires onero-sdk[sim] / mujoco") from exc
            if self.config.simulation_initial is not None:
                initial = np.asarray(self.config.simulation_initial)
                bounds = np.asarray(self.limits)
                if np.any(initial < bounds[:, 0]) or np.any(initial > bounds[:, 1]):
                    raise ValueError("simulation_initial is outside SDK joint limits")
        if self.config.backend == "can":
            try:
                import pinocchio  # noqa: F401
            except ImportError as exc:
                raise RuntimeError(
                    "CAN operation requires SDK dynamics / pinocchio for gravity compensation"
                ) from exc
            if _is_slcan_channel(self.config.channel):
                try:
                    import can  # noqa: F401
                    import serial  # noqa: F401
                    from onero_sdk import PythonCanBus  # noqa: F401
                except ImportError as exc:
                    raise RuntimeError(
                        "Direct SLCAN requires onero-sdk[python-can] and pyserial"
                    ) from exc
                if "buffered_slcan" not in signature(PythonCanBus).parameters:
                    raise RuntimeError(
                        "Direct SLCAN requires an updated SDK with buffered_slcan support"
                    )
                if not Path(self.config.channel).is_char_device():
                    raise FileNotFoundError(f"SLCAN serial device not found: {self.config.channel}")

    @property
    def connected(self) -> bool:
        return self._state is not None if self.config.backend == "mock" else self.arm is not None

    def _on_error(self, error) -> None:
        self._fault_detail = str(error)
        self._fault.set()

    @property
    def fault_reason(self) -> str | None:
        return self._fault_detail if self._fault.is_set() else None

    @property
    def gripper_fault(self) -> str | None:
        """Latched gripper fault, kept separate from the arm fault so that the arm keeps holding."""
        return self._gripper_fault

    def connect(self, *, leader: bool = False, damping: float = 0.3) -> None:
        for attempt in range(1, _CONNECT_ATTEMPTS + 1):
            try:
                self._connect_once(leader=leader, damping=damping)
                return
            except Exception as exc:
                if attempt >= _CONNECT_ATTEMPTS or not _retryable_connect_failure(exc):
                    raise
                logger.warning(
                    "Retrying %s (%s) connect after a lost pre-enable motor state "
                    "check (attempt %d of %d): %s",
                    self.config.model, self.config.channel or self.config.backend,
                    attempt + 1, _CONNECT_ATTEMPTS, exc,
                )
                time.sleep(_CONNECT_RETRY_DELAY_S)

    def _connect_once(self, *, leader: bool = False, damping: float = 0.3) -> None:
        label = f"{self.config.model} ({self.config.channel or self.config.backend})"
        with startup_stage(logger, f"{label} SDK dependencies"):
            self.preflight()
        self._fault.clear()
        self._fault_hold_reported = False
        self._source_stamp = None
        self._leader = leader
        self._started = self._host_update = time.monotonic()
        if self.config.backend == "mock":
            self._set_mock(self.config.mock_initial)
            self._held = None
            if self.config.gripper_channel:
                # Explicit rather than a silent default, so a config that turns the
                # channel off cannot keep reporting a gripper feature.
                self._gripper = _MockGripper(self.config, leader=leader)
                self._gripper.enable()
            return
        from onero_sdk import Arm

        kwargs = dict(
            # MuJoCo physics stays in its native base frame; retain config.mounting
            # separately for the combined viewer's installation orientation.
            mounting=None if self.config.backend == "mujoco" else self.config.mounting,
            with_gripper=self.config.with_gripper,
            on_error=self._on_error,
        )
        if self.config.gripper_channel:
            # Only a leader that declares measured handle endpoints is a
            # hand-moved teach handle. Any other arm -- including a leader that
            # drives one arm from another, as in the single-arm follow configs --
            # keeps the stock gripper driver and is enabled together with its arm.
            teach_handle = leader and self.config.teach_handle
            gripper = _build_gripper(self.config, handle_calibration=teach_handle)
            # A leader mirrors its gripper opening into the follower targets. Only
            # drivers with a passive status query can report a hand-moved gripper;
            # a silent 0.0 (fully closed) would otherwise slam the follower shut.
            if leader and not callable(getattr(gripper, "refresh", None)):
                raise ValueError(
                    f"The {self.config.gripper_protocol} gripper driver has no passive "
                    "status query, so it cannot be a teleoperation leader; use "
                    "gripper_protocol=\"onero\" or disable the gripper channel with "
                    "gripper_protocol=\"none\""
                )
            kwargs["end_effector"] = gripper
        slcan = self.config.backend == "can" and _is_slcan_channel(self.config.channel)
        if self.config.backend == "can":
            if not slcan:
                _log_can_queue(self.config.channel)
            logger.info(
                "Connecting %s (%s): mounting=%s with_gripper=%s gripper=%s/%s",
                self.config.model, self.config.channel,
                self.config.mounting, self.config.with_gripper,
                self.config.gripper_protocol if self.config.gripper_channel else "none",
                self.config.gripper_model,
            )
            if not slcan:
                kwargs["channel"] = self.config.channel
        else:
            self._simulation_backend = _mujoco_backend(self.config, self._simulation_control_hz)
            kwargs["backend"] = self._simulation_backend
        serial_bus = None
        try:
            if slcan:
                from onero_sdk import PythonCanBus

                with startup_stage(logger, f"{label} SLCAN transport"):
                    serial_bus = PythonCanBus(
                        channel=self.config.channel, interface="slcan",
                        bitrate=1_000_000, tty_baudrate=1_000_000,
                        buffered_slcan=True,
                    )
                kwargs["can_bus"] = serial_bus
            with startup_stage(logger, f"{label} Arm construction"):
                self.arm = Arm(self.config.model, **kwargs)
                self._gripper = self.arm.gripper
            with startup_stage(logger, f"{label} enable and initial feedback"):
                self.arm.enable()
            with startup_stage(logger, f"{label} {'freedrive' if leader else 'initial hold'}"):
                if leader and self.config.backend == "can":
                    self.resume_leader(damping)
                else:
                    target = self._bounded_hold_target(self.arm.state.joints)
                    self._servo(target)
                    self._held = None
            self._simulation_target = joint_vector(self.arm.state.joints)
            self._simulation_anchor = self._simulation_target.copy()
            self._simulation_auto = True
            self.snapshot(1.0)
        except BaseException:
            unowned_bus = serial_bus if self.arm is None else None
            # Cleanup must never replace the original connection error: both the
            # reported reason and the pre-enable retry decision depend on it.
            try:
                self.close()
            except BaseException:
                logger.exception("%s cleanup after a failed connect did not finish", label)
            finally:
                if unowned_bus is not None:
                    try:
                        unowned_bus.close()
                    except BaseException:
                        logger.exception("%s serial bus cleanup after a failed connect failed", label)
            raise

    def _set_mock(self, values) -> None:
        self._state = SimpleNamespace(
            joints=joint_vector(values).tolist(),
            velocities=[0.0] * 7,
            torques=[0.0] * 7,
            timestamp=time.monotonic(),
        )

    def snapshot(self, stale_s: float) -> SimpleNamespace:
        if not self.connected:
            raise ConnectionError("SDK arm is not connected")
        if self._fault.is_set():
            raise RuntimeError(f"SDK fault: {self._fault_detail}")
        now = time.monotonic()
        if self.config.backend == "mock":
            if self._leader and self.config.mock_motion and self._held is None:
                phase = now - self._started
                motion = 0.05 * np.sin(phase * 1.2) * np.linspace(1.0, 0.4, 7)
                self._set_mock(np.asarray(self.config.mock_initial) + motion)
            else:
                self._set_mock(self._state.joints)
            state = self._state
        else:
            if not self.arm.enabled:
                raise RuntimeError("SDK arm is no longer enabled")
            if (
                self.config.backend == "can"
                and self._leader
                and self._held is None
                and not self.arm.is_freedrive
            ):
                raise RuntimeError("Leader freedrive has stopped")
            state = self.arm.state
        q = joint_vector(state.joints)
        stamp = float(state.timestamp)
        if not math.isfinite(stamp):
            raise RuntimeError("Invalid SDK state timestamp")
        if self.config.backend == "mujoco":
            # SDK MuJoCo timestamps use simulation time, not host monotonic time.
            if stamp != self._source_stamp:
                self._host_update = now
            age = now - self._host_update
        else:
            age = now - stamp
        self._source_stamp = stamp
        if age < -0.01 or age > stale_s:
            raise RuntimeError(f"SDK state is stale (age={age:.3f}s)")
        gripper = self._read_gripper() if self._gripper is not None else None
        return SimpleNamespace(
            joints=q.tolist(), timestamp=stamp, host_time=now, gripper=gripper,
        )

    def _latch_gripper_fault(self, detail: str) -> None:
        """Latch a gripper fault separately from the arm fault and report it once."""
        if self._gripper_fault != detail:
            self._gripper_fault = detail
            logger.error(
                "%s on %s (%s); gripper commands are blocked while it is reported",
                detail, self.config.model, self.config.channel,
            )

    def _read_gripper(self) -> float:
        state = self._gripper.state
        now = time.monotonic()
        # Only the Onero driver defines a passive status query. OmniPicker has no
        # query frame in the SDK driver, so it reports its last command/feedback
        # instead of a live reading; mirroring a hand-moved leader therefore
        # requires the Onero protocol.
        refresh = getattr(self._gripper, "refresh", None)
        query_failed = False
        if (
            refresh is not None
            and now - state.timestamp > _GRIPPER_FRESH_S
            and now - self._gripper_query >= _GRIPPER_QUERY_S
        ):
            self._gripper_query = now
            try:
                refresh()
            except Exception as exc:
                # Raising here would surface as an arm hold failure, because
                # snapshot() also runs inside command()/hold().
                query_failed = True
                self._gripper_query_failures += 1
                limit = self.config.gripper_query_failures_before_fault
                if self._gripper_query_failures >= limit:
                    self._latch_gripper_fault(
                        f"Gripper status query failed {self._gripper_query_failures} "
                        f"times in a row: {exc}"
                    )
                else:
                    # One lost frame on a slcan/USB link must not stop the
                    # session; the previous reading stays in use meanwhile.
                    logger.warning(
                        "Gripper status query on %s (%s) missed (%d/%d); keeping the "
                        "last reading: %s",
                        self.config.model, self.config.channel or self.config.backend,
                        self._gripper_query_failures, limit, exc,
                    )
            else:
                state = self._gripper.state
                self._gripper_query_failures = 0
        # A gripper fault is deliberately kept out of the arm fault path: the SDK
        # arm is healthy and must keep holding, while hold() after an arm fault
        # reports POSITION HOLD UNAVAILABLE. It is latched here instead, and
        # gripper_fault stops the session from the follower/leader control path.
        if state.fault_code:
            self._latch_gripper_fault(
                f"Gripper fault: protocol={self.config.gripper_protocol} "
                f"code={state.fault_code} state={state.motion_state}"
            )
        elif not query_failed and self._gripper_fault is not None:
            self._gripper_fault = None
            logger.info("Gripper fault cleared on %s (%s)", self.config.model, self.config.channel)
        if self._gripper_commanded and state.timestamp == 0.0 and not self._gripper_silent:
            # OmniPicker tolerates a missing reply, so a gripper that never
            # answered would otherwise keep reporting its commanded target as if
            # it were measured feedback.
            self._gripper_silent = True
            logger.warning(
                "Gripper on %s (%s) has not returned any feedback since the first "
                "command; readings are the commanded target, not a measured opening",
                self.config.model, self.config.channel,
            )
        return float(self._gripper.get_position())

    def gripper_command(self, position: float) -> None:
        """Command the gripper channel; normalized 1.0 = open, 0.0 = closed."""
        if self._gripper is None:
            raise RuntimeError("This arm has no gripper channel")
        position = float(position)
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("Gripper position must be finite and in [0.0, 1.0]")
        if self._fault.is_set():
            raise RuntimeError(f"SDK fault: {self._fault_detail}")
        if self._gripper_fault is not None:
            raise RuntimeError(self._gripper_fault)
        try:
            self._gripper.set_position(position)
        except RuntimeError:
            # The Onero driver re-derives _enabled from the status nibble of every
            # feedback frame, so one bad or foreign frame is enough to make a
            # healthy motor look disabled and have it reject the next position
            # frame; that used to surface as a latched following fault on a
            # working arm. Re-assert enable once (idempotent) and retry. The
            # warning keeps the frequency visible, and a motor that really is dead
            # still fails on the retry.
            if self._leader or getattr(self._gripper, "_enabled", True) is not False:
                raise
            logger.warning(
                "Gripper on %s (%s) reported disabled in feedback; re-asserting "
                "enable and retrying the position command",
                self.config.model, self.config.channel or self.config.backend,
            )
            self._gripper.enable()
            self._gripper.set_position(position)
        self._gripper_commanded = True

    def query(self, stale_s: float) -> SimpleNamespace:
        """Explicit passive observation query; active servo uses its cached reply."""
        if self.arm is not None:
            self.arm.get_state()
            if self.config.backend == "mujoco":
                self._host_update = time.monotonic()
        return self.snapshot(stale_s)

    def command(self, positions, velocities=None) -> SimpleNamespace:
        if self._fault.is_set():
            raise RuntimeError(f"SDK fault: {self._fault_detail}")
        if self.config.backend == "mock":
            # Validate before updating mock state, just as the real servo does.
            _joint_velocities(velocities)
            self._set_mock(positions)
        else:
            self._servo(positions, velocities)
        return self.snapshot(1.0)

    def _servo(self, positions, velocities=None) -> None:
        # None deliberately uses SDK's per-call zero-velocity default. Keep no
        # velocity cache: initial holding, pauses and position-only callers
        # must never inherit a prior moving command's feedforward.
        q = joint_vector(positions).tolist()
        velocity = _joint_velocities(velocities)
        kwargs = {} if velocity is None else {"velocities": velocity}
        if self.config.backend == "mujoco":
            # MuJoCo already computes C(q, qdot). Pass this through Arm's public
            # feed-forward API so simulated arms do not sag without Pinocchio.
            # CAN continues to use the SDK's calibrated gravity compensation.
            _, data = self.mujoco_scene()
            self.arm.servoj(q, tau_ff=data.qfrc_bias.tolist(), **kwargs)
        else:
            self.arm.servoj(q, **kwargs)

    def mujoco_scene(self):
        """Return the *actual* SDK scene used for this device, for passive viewing.

        The SDK's own viewer examples expose these two renderer attributes. Keep
        this small compatibility seam here; motion still goes through Arm APIs.
        """
        backend = self._simulation_backend
        if backend is None or not self.connected:
            raise RuntimeError("A connected SDK MuJoCo backend is required for viewing")
        try:
            return backend._model, backend._data
        except AttributeError as exc:
            raise RuntimeError("This SDK version does not expose the MuJoCo viewer scene") from exc

    def mujoco_model_path(self) -> str:
        if self._simulation_backend is None:
            raise RuntimeError("A connected SDK MuJoCo backend is required")
        return self._simulation_backend._find_urdf(self.config.model)

    @property
    def simulation_auto(self) -> bool:
        return self._simulation_auto

    def step_simulation_leader(self) -> None:
        if self.config.backend != "mujoco" or not self._leader or self._held is not None:
            return
        if self._simulation_auto:
            elapsed = time.monotonic() - self._started
            wave = 0.15 * math.sin(0.8 * elapsed)
            direction = 1 if self.config.model == "A1L" else -1
            self._simulation_target = self._simulation_anchor + direction * wave * np.array(
                [1.0, 0.5, 0.7, 0.9, 0.3, 0.5, 0.3]
            )
        bounds = np.asarray(self.limits)
        self.command(np.clip(self._simulation_target, bounds[:, 0], bounds[:, 1]))

    def set_simulation_auto(self, enabled: bool) -> None:
        self._require_simulation_leader()
        self._simulation_target = joint_vector(self.arm.state.joints)
        self._simulation_anchor = self._simulation_target.copy()
        self._started = time.monotonic()
        self._simulation_auto = enabled

    def jog_simulation_joint(self, joint: int, delta: float) -> None:
        self._require_simulation_leader()
        if self._held is not None:
            raise RuntimeError("Resume following before jogging a simulated leader")
        if joint not in range(7) or not math.isfinite(delta):
            raise ValueError("A joint index 0..6 and finite delta are required")
        if self._simulation_auto:
            self.set_simulation_auto(False)
        lower, upper = self.limits[joint]
        self._simulation_target[joint] = np.clip(
            self._simulation_target[joint] + delta, lower, upper
        )

    def _require_simulation_leader(self) -> None:
        if self.config.backend != "mujoco" or not self._leader or not self.connected:
            raise RuntimeError("This operation only controls a connected MuJoCo leader")

    def hold(self) -> bool:
        """Stop once, then refresh a held target on subsequent control ticks."""
        if not self.connected:
            return False
        if self._fault.is_set():
            # The SDK callback reports a latched emergency stop. Position
            # commands are forbidden until an explicit reconnect; cleanup must
            # still reach close(disable=True) for this and every other arm.
            if not self._fault_hold_reported:
                logger.warning(
                    "Skipping position hold for %s (%s) after SDK fault; "
                    "keeping emergency stop latched: %s",
                    self.config.model,
                    self.config.channel or self.config.backend,
                    self._fault_detail,
                )
                self._fault_hold_reported = True
            return False
        if self._held is None:
            if self.config.backend == "mock":
                self._held = list(self._state.joints)
            elif self.config.backend == "mujoco":
                # Hold through the same compensated servo path; SDK stop() uses
                # its separate optional Pinocchio path. Do not change CAN stop.
                self._held = list(self.arm.state.joints)
            else:
                if self._leader and self.arm.is_freedrive:
                    self.arm.freedrive_disable()
                else:
                    self.arm.stop()
                self._held = list(self.arm.state.joints)
            self._held = self._bounded_hold_target(self._held)
        self.command(self._held)
        return True

    def _bounded_hold_target(self, positions) -> list[float]:
        bounds = np.asarray(self.limits)
        return np.clip(joint_vector(positions), bounds[:, 0], bounds[:, 1]).tolist()

    def resume_leader(self, damping: float) -> None:
        if self.config.backend == "can":
            logger.info(
                "Starting freedrive for %s (%s) at SDK %.1f Hz; "
                "this background rate is independent of the session following rate",
                self.config.model, self.config.channel, self.arm.control_freq,
            )
            self.arm.freedrive_enable(damping=damping)
        elif self.config.backend == "mujoco":
            self.set_simulation_auto(self._simulation_auto)
        self._held = None

    def clear_hold(self) -> None:
        self._held = None

    def close(self) -> None:
        try:
            if self.arm is not None:
                logger.info(
                    "Disabling and closing %s (%s)",
                    self.config.model,
                    self.config.channel or self.config.backend,
                )
                # The application explicitly requests torque-off on disconnect.
                # SDK close stops freedrive before disabling and detaching.
                self.arm.close(disable=True)
        finally:
            self.arm = None
            self._gripper = None
            self._gripper_commanded = False
            self._gripper_silent = False
            self._gripper_fault = None
            self._gripper_query_failures = 0
            self._state = None
            self._held = None
            self._simulation_backend = None


def close_devices(devices: dict[str, SdkDevice]) -> None:
    for name, device in reversed(list(devices.items())):
        try:
            device.close()
        except Exception:
            logger.exception("Could not close arm %s", name)
    devices.clear()


def hold_devices(devices: dict[str, SdkDevice]) -> None:
    errors = []
    for name, device in devices.items():
        try:
            device.hold()
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    if errors:
        raise RuntimeError("Could not hold all arms: " + "; ".join(errors))
