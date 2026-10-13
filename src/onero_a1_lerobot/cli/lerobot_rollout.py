"""Register the A1 SDK plugin and delegate to LeRobot's policy rollout CLI."""

from __future__ import annotations

import sys


def _ensure_sdk_robot_type(argv: list[str]) -> None:
    found = False
    for index, argument in enumerate(argv[1:], start=1):
        if argument == "--robot.type":
            if index + 1 >= len(argv) or argv[index + 1].startswith("--"):
                raise SystemExit("ERROR: --robot.type requires onero_sdk")
            robot_type = argv[index + 1]
        elif argument.startswith("--robot.type="):
            robot_type = argument.partition("=")[2]
        else:
            continue
        if robot_type != "onero_sdk":
            raise SystemExit("ERROR: A1 rollout requires --robot.type=onero_sdk")
        found = True
    if not found:
        argv.append("--robot.type=onero_sdk")


def main() -> None:
    # Register SDK configs only in the CLI, before LeRobot parses them. Spawned
    # CAN workers may import this entrypoint without needing LeRobot themselves.
    from onero_a1_lerobot.sdk_config import OneroSdkConfig, OneroSdkTeleopConfig  # noqa: F401

    _ensure_sdk_robot_type(sys.argv)
    try:
        from lerobot.scripts.lerobot_rollout import main as lerobot_rollout_main
    except Exception:
        raise SystemExit(
            "ERROR: LeRobot is not available. Install LeRobot support with: "
            "python3 -m pip install -e '.[lerobot]'"
        ) from None

    lerobot_rollout_main()


if __name__ == "__main__":
    main()
