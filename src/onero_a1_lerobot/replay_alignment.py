"""Bounded start-pose alignment for replay, using the normal follower path."""

from __future__ import annotations

import logging
import math
import time

import numpy as np

from .arm_mapping import JointSlewLimiter, joint_vector
from .sdk_device_config import positive

logger = logging.getLogger(__name__)

_PEAK_BLEND_VELOCITY = 1.875
_PEAK_BLEND_ACCELERATION = 10.0 / math.sqrt(3.0)
_GRIPPER_VELOCITY = 0.25  # Normalized opening units per second.
_GRIPPER_ACCELERATION = 0.5
_GRIPPER_TOLERANCE = 0.03
_SETTLE_TIME_S = 0.25
_JOINT_STABILITY_RAD = 0.005
_GRIPPER_STABILITY = 0.01


def _duration_for_delta(delta, velocity, acceleration) -> float:
    """Duration bounds for s(u)=10u^3-15u^4+6u^5, u in [0,1]."""
    distance = np.abs(delta)
    return max(
        float(np.max(_PEAK_BLEND_VELOCITY * distance / velocity)),
        float(np.max(np.sqrt(_PEAK_BLEND_ACCELERATION * distance / acceleration))),
    )


def align_to_first_frame(
    robot, target: dict[str, float], *, minimum_duration_s: float = 3.0,
    max_velocity: float = 0.2, max_acceleration: float = 0.5,
    settle_timeout_s: float = 15.0, joint_tolerance_rad: float | None = None,
) -> None:
    """Move all follower joints/grippers to frame zero, then restore replay limits.

    This is joint-space interpolation, not collision-free path planning. Every
    sample goes through send_action, including its initial-distance, stale-state,
    tracking-error and control-gap guards. The first sample is the measured pose.
    """
    for name, value in (
        ("align_duration", minimum_duration_s), ("align_max_velocity", max_velocity),
        ("align_max_acceleration", max_acceleration), ("align_timeout", settle_timeout_s),
    ):
        positive(value, name)
    if joint_tolerance_rad is not None:
        positive(joint_tolerance_rad, "align_tolerance")
    if not robot.is_connected or not robot.config.send_action:
        raise RuntimeError("Replay alignment requires connected, enabled follower control")
    if set(target) != set(robot.action_features) or not all(math.isfinite(v) for v in target.values()):
        raise ValueError("First replay action must contain every configured finite action feature")

    try:
        # Keep process followers refreshing a stationary hold while preparing
        # and validating every arm. No transition target is sent until all pass.
        robot.hold()
        initial = robot.get_joint_observation()
        start = {name: float(initial[name]) for name in target}
        if not all(math.isfinite(value) for value in start.values()):
            raise ValueError("Cannot align from non-finite follower feedback")
        limiters = {}
        replay_limiters = robot._limiters
        joint_keys = []
        duration = minimum_duration_s
        for side, keys in robot._keys_by_side.items():
            joint_keys.extend(keys)
            q0 = joint_vector([start[key] for key in keys])
            q1 = joint_vector([target[key] for key in keys])
            previous = robot._limiters[side]
            bounds = previous.limits
            for label, position in (("measured start", q0), ("first replay target", q1)):
                outside = np.flatnonzero((position < bounds[:, 0]) | (position > bounds[:, 1]))
                if len(outside):
                    details = "; ".join(
                        f"J{i + 1}={position[i]:.4f} outside [{bounds[i, 0]:.4f}, {bounds[i, 1]:.4f}] rad"
                        for i in outside
                    )
                    raise ValueError(f"Cannot align {side}: {label}: {details}")
            # Tighten the actual command limiter as well as the reference
            # trajectory; a delayed tick must not cause a fast catch-up move.
            limiter = JointSlewLimiter(
                q0, bounds,
                np.minimum(
                    np.minimum(previous.joint_max_velocity, robot.devices[side].joint_velocity_limits),
                    max_velocity,
                ),
                np.minimum(previous.joint_max_acceleration, max_acceleration),
                continuous=previous.continuous,
                max_step=robot.devices[side].servo_max_step,
                clip_joint_limits=True,
            )
            limiters[side] = limiter
            duration = max(duration, _duration_for_delta(
                q1 - q0, limiter.joint_max_velocity, limiter.joint_max_acceleration,
            ))
            logger.info(
                "Replay alignment %s: current=%s target=%s max_delta=%.4f rad",
                side, np.round(q0, 4).tolist(), np.round(q1, 4).tolist(), float(np.max(abs(q1 - q0))),
            )
        gripper_keys = tuple(robot._gripper_key_by_side.values())
        for key in gripper_keys:
            if not (0.0 <= start[key] <= 1.0 and 0.0 <= target[key] <= 1.0):
                raise ValueError(f"Cannot align {key}: feedback and target must be in [0, 1]")
            duration = max(duration, _duration_for_delta(
                target[key] - start[key], _GRIPPER_VELOCITY, _GRIPPER_ACCELERATION,
            ))
        if not math.isfinite(duration):
            raise ValueError("Replay alignment duration is not finite; check the configured limits")

        # Use the same admissible distance as the recorded first-frame guard.
        # An explicit option may tighten it, but cannot bypass that guard.
        # Tracking-error protection remains active inside every send_action.
        tolerance = robot.config.startup_max_delta_rad
        if joint_tolerance_rad is not None:
            tolerance = min(joint_tolerance_rad, tolerance)
        stable_joint_range = min(_JOINT_STABILITY_RAD, tolerance * 0.25)
        period = 1.0 / robot.config.control_hz
        logger.info(
            "Aligning replay start: duration=%.2fs max_velocity=%.3f rad/s "
            "max_acceleration=%.3f rad/s^2 tolerance=%.4f rad; then replay begins",
            duration, max_velocity, max_acceleration, tolerance,
        )
        robot._limiters = limiters
        for device in robot.devices.values():
            device.clear_hold()
        # Retain the normal first-send guard: command measured positions first.
        robot.send_action(start)
        started = time.monotonic()
        deadline = started + duration + settle_timeout_s
        stable_since = None
        stable_min = stable_max = None
        next_report = started + 2.0
        delta = {name: target[name] - start[name] for name in target}
        while True:
            tick_started = time.monotonic()
            u = min(1.0, max(0.0, (tick_started - started) / duration))
            blend = min(1.0, max(0.0, u * u * u * (10.0 + u * (-15.0 + 6.0 * u))))
            action = dict(target) if u >= 1.0 else {
                name: start[name] + blend * delta[name] for name in target
            }
            submitted = robot.send_action(action)
            feedback = robot.get_joint_observation()
            if not all(math.isfinite(float(feedback[name])) for name in target):
                raise RuntimeError("Non-finite follower feedback during replay alignment")
            joint_errors = {key: abs(float(feedback[key]) - target[key]) for key in joint_keys}
            worst_joint = max(joint_errors, key=joint_errors.get)
            error = joint_errors[worst_joint]
            command_error = max(abs(submitted[key] - target[key]) for key in joint_keys)
            gripper_error = max((abs(float(feedback[key]) - target[key]) for key in gripper_keys), default=0.0)
            velocity = max(float(np.max(abs(limiter.velocity))) for limiter in limiters.values())
            now = time.monotonic()
            if (u >= 1.0 and error <= tolerance and command_error <= min(0.001, tolerance * 0.25)
                    and gripper_error <= _GRIPPER_TOLERANCE and velocity <= min(0.01, max_velocity * 0.1)):
                if stable_since is None:
                    stable_since = now
                    stable_min = {key: float(feedback[key]) for key in target}
                    stable_max = dict(stable_min)
                else:
                    for key in target:
                        stable_min[key] = min(stable_min[key], float(feedback[key]))
                        stable_max[key] = max(stable_max[key], float(feedback[key]))
                    joint_range = max(stable_max[key] - stable_min[key] for key in joint_keys)
                    gripper_range = max((stable_max[key] - stable_min[key] for key in gripper_keys), default=0.0)
                    if joint_range > stable_joint_range or gripper_range > _GRIPPER_STABILITY:
                        # Near the target is insufficient if measured motion
                        # or oscillation continues. Restart the stability window.
                        stable_since = now
                        stable_min = {key: float(feedback[key]) for key in target}
                        stable_max = dict(stable_min)
                if now - stable_since >= _SETTLE_TIME_S:
                    break
            else:
                stable_since = None
            if now >= deadline:
                raise TimeoutError(
                    f"Replay alignment did not settle within {duration + settle_timeout_s:.2f}s: "
                    f"joint_error={error:.4f} rad, command_error={command_error:.4f} rad, "
                    f"gripper_error={gripper_error:.4f}, worst_joint={worst_joint}, "
                    f"target={target[worst_joint]:.4f}, actual={float(feedback[worst_joint]):.4f}, "
                    f"tolerance={tolerance:.4f} rad; episode not started"
                )
            if now >= next_report:
                logger.info(
                    "Replay alignment: elapsed=%.2fs joint_error=%.4f rad worst_joint=%s "
                    "target=%.4f actual=%.4f command_error=%.4f rad gripper_error=%.4f stable_for=%.2fs",
                    now - started, error, worst_joint, target[worst_joint], float(feedback[worst_joint]),
                    command_error, gripper_error, 0.0 if stable_since is None else now - stable_since,
                )
                next_report = now + 2.0
            time.sleep(max(0.0, period - (time.monotonic() - tick_started)))

        # Keep the last commanded position during handover. Re-holding at the
        # measured position would shift the target by any steady tracking error.
        # Carry the small settled command velocity into the normal replay limits.
        restored = {}
        for side, previous in replay_limiters.items():
            aligned = limiters[side]
            restored[side] = JointSlewLimiter(
                aligned.position, previous.limits,
                previous.joint_max_velocity, previous.joint_max_acceleration,
                continuous=previous.continuous, max_step=previous.max_step,
                clip_joint_limits=previous.clip_joint_limits,
            )
            restored[side].velocity = aligned.velocity.copy()
        robot._limiters = restored
        robot._first_send = True
        # Keep _last_send intact: a delayed handover must still trigger the
        # control-gap watchdog. The first recorded action rechecks real feedback.
        logger.info(
            "Replay start aligned in %.2fs; max_joint_error=%.4f rad worst_joint=%s tolerance=%.4f rad",
            time.monotonic() - started, error, worst_joint, tolerance,
        )
    except BaseException:
        try:
            robot.hold()
        except Exception:
            logger.exception("Could not hold all followers after replay alignment failed")
        raise
