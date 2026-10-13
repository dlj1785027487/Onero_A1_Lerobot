"""Canonical A1 joint features, ordered left then right, with angles in radians."""

DEFAULT_LEFT_ARM_JOINTS = tuple(f"joint{i}-l" for i in range(1, 8))
DEFAULT_RIGHT_ARM_JOINTS = tuple(f"joint{i}-r" for i in range(1, 8))


def arm_position_keys(side: str, joint_names: tuple[str, ...] | None = None) -> tuple[str, ...]:
    if side not in ("left", "right"):
        raise ValueError(f"Unknown arm side: {side!r}")
    if joint_names is None:
        joint_names = DEFAULT_LEFT_ARM_JOINTS if side == "left" else DEFAULT_RIGHT_ARM_JOINTS
    return tuple(f"{side}_arm.{joint}.pos" for joint in joint_names)


def gripper_key(side: str) -> str:
    """Normalized gripper position, 1.0 = open and 0.0 = closed."""
    if side not in ("left", "right"):
        raise ValueError(f"Unknown arm side: {side!r}")
    return f"{side}_arm.gripper.pos"


def ordered_sides(arms) -> tuple[str, ...]:
    if not arms or set(arms) - {"left", "right"}:
        raise ValueError("Configure one or both logical arm sides: left, right")
    return tuple(side for side in ("left", "right") if side in arms)
