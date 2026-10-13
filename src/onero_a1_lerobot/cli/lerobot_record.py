"""Record A1 CAN leader/follower sessions as LeRobot datasets."""

from __future__ import annotations

import sys


def main() -> None:
    from onero_a1_lerobot.sdk_session import run_session_cli

    run_session_cli(sys.argv[1:], record=True)


if __name__ == "__main__":
    main()
