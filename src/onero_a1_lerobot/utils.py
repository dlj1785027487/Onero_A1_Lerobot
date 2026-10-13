"""Scalar and action conversions for the A1 CAN adapter."""

from __future__ import annotations

from typing import Any

import numpy as np


def to_float(value: Any) -> float:
    """Convert Python, NumPy, or torch-like scalar values to float."""

    if hasattr(value, "detach") and hasattr(value, "cpu"):
        value = value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        if value.size != 1:
            raise ValueError(f"Expected scalar value, got array shape {value.shape}")
        return float(value.reshape(-1)[0])
    return float(value)


def normalize_action_dict(action: dict[str, Any], names: tuple[str, ...]) -> dict[str, float]:
    """Normalize LeRobot action input.

    LeRobot usually passes a flat dict whose keys match ``action_features``. For
    convenience, this function also accepts ``{"action": vector}`` and expands it
    using the configured feature-name order.
    """

    if "action" in action and len(action) == 1:
        values = action["action"]
        if hasattr(values, "detach") and hasattr(values, "cpu"):
            values = values.detach().cpu().numpy()
        values = np.asarray(values, dtype=np.float32).reshape(-1)
        if values.shape[0] != len(names):
            raise ValueError(f"Action vector has {values.shape[0]} values, expected {len(names)}")
        return {name: float(values[i]) for i, name in enumerate(names)}

    return {key: to_float(value) for key, value in action.items()}
