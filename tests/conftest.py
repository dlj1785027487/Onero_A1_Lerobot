"""Shared fixtures for the A1 LeRobot adapter tests.

Every test here must run without CAN, USB cameras, or the onero SDK service:
that is the whole point of the configuration and encoding coverage.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "configs"


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def config_dir() -> Path:
    return CONFIG_DIR


def read_config(config_dir: Path, name: str) -> dict:
    """Parse one JSON config without applying defaults."""
    return json.loads((config_dir / name).read_text(encoding="utf-8"))
