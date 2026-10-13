"""Compatibility layer for using this package with or without LeRobot installed.

The real target runtime is LeRobot. The small fallback classes below only make the
package importable for CAN teleoperation and unit tests without LeRobot installed.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TypeAlias

logger = logging.getLogger(__name__)

try:  # pragma: no cover - depends on the user's LeRobot installation
    from lerobot.robots.config import RobotConfig
    from lerobot.robots.robot import Robot
    from lerobot.teleoperators.config import TeleoperatorConfig
    from lerobot.teleoperators.teleoperator import Teleoperator

    # LeRobot renamed ``lerobot.types`` to ``lerobot.lerobot_types`` in 0.6.1.
    # The project pins 0.6.1, while the fallback keeps older developer
    # environments importable long enough to report a useful compatibility error.
    try:
        from lerobot.lerobot_types import RobotAction, RobotObservation
    except ImportError:  # pragma: no cover - compatibility with LeRobot <= 0.6.0
        from lerobot.types import RobotAction, RobotObservation

    LEROBOT_AVAILABLE = True
except Exception as exc:  # pragma: no cover - exercised when LeRobot is not installed
    # Nothing should be silent here: a broken LeRobot installation must not look
    # like "LeRobot is not installed" and send the user chasing the wrong cause.
    logger.warning(
        "LeRobot is unavailable (%s: %s); falling back to the importable stubs",
        type(exc).__name__, exc,
    )
    LEROBOT_AVAILABLE = False

    # Keep assignment syntax for Python 3.10 compatibility.
    RobotAction: TypeAlias = dict[str, Any]  # noqa: UP040
    RobotObservation: TypeAlias = dict[str, Any]  # noqa: UP040

    @dataclass(kw_only=True)
    class RobotConfig:
        """Minimal fallback matching the fields used by LeRobot's RobotConfig."""

        id: str | None = None
        calibration_dir: Path | None = None

        @property
        def type(self) -> str:
            return self.__class__.__name__

    @dataclass(kw_only=True)
    class TeleoperatorConfig:
        """Minimal fallback matching the fields used by LeRobot's TeleoperatorConfig."""

        id: str | None = None
        calibration_dir: Path | None = None

        @property
        def type(self) -> str:
            return self.__class__.__name__

    class Robot:
        """Minimal fallback base class.

        This is intentionally small. It is not a replacement for LeRobot; install
        LeRobot for recording, training, and policy evaluation.
        """

        config_class: type[RobotConfig]
        name: str

        def __init__(self, config: RobotConfig):
            self.robot_type = self.name
            self.id = config.id
            self.calibration_dir = config.calibration_dir
            self.calibration = {}

        def __enter__(self):
            self.connect()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self.disconnect()

    class Teleoperator:
        """Minimal fallback base class for import-time tests without LeRobot."""

        config_class: type[TeleoperatorConfig]
        name: str

        def __init__(self, config: TeleoperatorConfig):
            self.id = config.id
            self.calibration_dir = config.calibration_dir
            self.calibration = {}

        def __enter__(self):
            self.connect()
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self.disconnect()
