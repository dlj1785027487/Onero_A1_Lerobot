"""Pure joint mapping and time-based motion limiting, independent of transports."""

from __future__ import annotations

import math
from numbers import Real

import numpy as np

from .sdk_device_config import ArmMappingConfig, positive


def joint_vector(values) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError("Expected seven finite joint angles in radians")
    return result.copy()


def _joint_positive_vector(values, name: str) -> np.ndarray:
    """Copy a positive scalar or seven positive joint limits without coercing bools."""
    message = f"{name} must be a finite positive number or seven finite positive numbers"
    try:
        raw = np.asarray(values, dtype=object)
        if raw.shape == ():
            items = [raw.item()] * 7
        elif raw.shape == (7,):
            items = raw.tolist()
        else:
            raise ValueError(message)
        if any(isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) for value in items):
            raise ValueError(message)
        result = np.asarray(items, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(message) from exc
    if not np.all(np.isfinite(result)) or np.any(result <= 0):
        raise ValueError(message)
    return result


class ArmMapper:
    def __init__(self, config: ArmMappingConfig):
        config.__post_init__()
        self.config = config
        self._leader_ref: np.ndarray | None = None
        self._follower_ref: np.ndarray | None = None

    def capture(self, leader, follower) -> None:
        self._leader_ref = joint_vector(leader)
        self._follower_ref = joint_vector(follower)

    def clear(self) -> None:
        self._leader_ref = self._follower_ref = None

    def map(self, leader) -> np.ndarray:
        q = joint_vector(leader)
        signs = (
            np.asarray(self.config.joint_signs)
            if self.config.mode == "joint"
            else -1
        )
        factor = self.config.scale * signs
        if self.config.reference == "relative":
            if self._leader_ref is None or self._follower_ref is None:
                raise RuntimeError("Relative mapping requires prepare() in a shared SDK session")
            return self._follower_ref + factor * (q - self._leader_ref)
        return factor * q + np.asarray(self.config.offset)


class JointSlewLimiter:
    def __init__(
        self, initial, limits,
        max_velocity: float | list[float] | tuple[float, ...] | np.ndarray,
        max_acceleration: float | list[float] | tuple[float, ...] | np.ndarray,
        *, continuous: bool = False, max_step: float | None = None,
        clip_joint_limits: bool = True,
    ):
        self.position = joint_vector(initial)
        self.velocity = np.zeros(7)
        self.limits = np.asarray(limits, dtype=float)
        if (
            self.limits.shape != (7, 2)
            or not np.all(np.isfinite(self.limits))
            or np.any(self.limits[:, 0] > self.limits[:, 1])
        ):
            raise ValueError("Joint limits must be seven finite [lower, upper] pairs")
        if not isinstance(clip_joint_limits, bool):
            raise ValueError("clip_joint_limits must be a boolean")
        self.clip_joint_limits = clip_joint_limits
        if clip_joint_limits and (
            np.any(self.position < self.limits[:, 0]) or np.any(self.position > self.limits[:, 1])
        ):
            raise ValueError("Initial position is outside joint limits")
        self.joint_max_velocity = _joint_positive_vector(max_velocity, "max_velocity")
        self.joint_max_acceleration = _joint_positive_vector(max_acceleration, "max_acceleration")
        self.continuous = continuous
        if max_step is not None:
            positive(max_step, "max_step")
            # A delayed tick must still allow deceleration without exceeding
            # the per-packet position bound. For any future dt > 0, the
            # smallest forward step is max(0, v - a*dt)*dt, whose maximum is
            # v**2/(4*a). Keep that <= max_step, with a rounding margin.
            step_safe_velocity = (
                2 * np.sqrt(self.joint_max_acceleration) * np.sqrt(max_step) * (1 - 1e-12)
            )
            self.joint_max_velocity = np.minimum(self.joint_max_velocity, step_safe_velocity)
        # Keep scalar summaries for existing diagnostics and callers.
        self.max_velocity = float(np.max(self.joint_max_velocity))
        self.max_acceleration = float(np.max(self.joint_max_acceleration))
        self.max_step = max_step
        self._previous_target: np.ndarray | None = None
        self._target_velocity = np.zeros(7)

    def _braking_velocity(self, distance: np.ndarray, dt: float) -> np.ndarray:
        # With q_next = q + dt * v_next, reserve both this step's travel and
        # the remaining stopping distance: dt*v + v**2/(2*a) <= distance.
        # Keeping this invariant at each boundary makes maximum deceleration
        # feasible on the next tick even if dt changes or the target reverses.
        # This form of the positive quadratic root avoids cancellation near 0.
        return 2 * distance / (np.sqrt(dt * dt + 2 * distance / self.joint_max_acceleration) + dt)

    def step(self, target, dt: float) -> np.ndarray:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Control dt must be finite and positive")
        requested = joint_vector(target)
        positions = self.position.tolist()
        velocities = self.velocity.tolist()
        limits = self.limits.tolist()
        target_velocities = self._target_velocity.tolist()
        previous = None if self._previous_target is None else self._previous_target.tolist()
        max_velocities = self.joint_max_velocity.tolist()
        accelerations = self.joint_max_acceleration.tolist()
        step_velocity = None if self.max_step is None else self.max_step / dt
        alpha = -math.expm1(-dt / 0.02) if self.continuous and previous is not None else None
        dt_squared = dt * dt

        def braking(distance):
            return 2 * distance / (math.sqrt(dt_squared + 2 * distance / acceleration) + dt)

        targets, next_velocities, next_positions, errors = [], [], [], []
        # Seven scalar joints avoid allocating many short-lived NumPy arrays.
        # Compute a complete candidate before committing any limiter state.
        for i, requested_q in enumerate(requested.tolist()):
            q, velocity = positions[i], velocities[i]
            max_velocity = max_velocities[i]
            acceleration = accelerations[i]
            acceleration_step = acceleration * dt
            lower, upper = limits[i]
            target_q = min(max(requested_q, lower), upper) if self.clip_joint_limits else requested_q
            targets.append(target_q)
            error = target_q - q
            sign = 1.0 if error > 0 else (-1.0 if error < 0 else 0.0)
            distance = abs(error)
            braking_speed = braking(distance)
            if self.continuous:
                if alpha is not None:
                    # Filter before clipping so repeated/quantized samples
                    # retain their average source speed.
                    target_velocities[i] = (
                        (1 - alpha) * target_velocities[i]
                        + (alpha / dt) * (target_q - previous[i])
                    )
                estimate = min(max(target_velocities[i], -max_velocity), max_velocity)
                approach = max(0.0, sign * estimate + braking_speed)
                desired = sign * min(distance / dt, approach)
            else:
                desired = sign * braking_speed
            v_min = max(-max_velocity, velocity - acceleration_step)
            v_max = min(max_velocity, velocity + acceleration_step)
            if self.clip_joint_limits:
                v_min = max(v_min, -braking(q - lower))
                v_max = min(v_max, braking(upper - q))
            if step_velocity is not None:
                # Position and velocity must obey the same per-packet bound.
                v_min = max(v_min, -step_velocity)
                v_max = min(v_max, step_velocity)
            if v_min > v_max:
                errors.append(
                    f"J{i + 1}: q={q:.9f} rad, v={velocity:.9f} rad/s, "
                    f"target={requested_q:.9f} rad, limits=[{lower:.9f}, {upper:.9f}], "
                    f"allowed_velocity=[{v_min:.9f}, {v_max:.9f}] rad/s"
                )
            next_velocity = min(max(desired, v_min), v_max)
            next_velocities.append(next_velocity)
            next_positions.append(q + dt * next_velocity)
        if errors:
            raise RuntimeError(
                "Cannot remain within motion limits at the configured deceleration: "
                f"max_step={self.max_step}; "
                f"dt={dt:.9f} s; {'; '.join(errors)}"
            )
        self.velocity = np.asarray(next_velocities)
        self.position = np.asarray(next_positions)
        self._previous_target = np.asarray(targets)
        self._target_velocity = np.asarray(target_velocities)
        return self.position.copy()
