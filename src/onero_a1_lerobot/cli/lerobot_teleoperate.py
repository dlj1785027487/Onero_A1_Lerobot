"""Run an A1 CAN leader/follower session."""

from __future__ import annotations

import sys


def main() -> None:
    from onero_a1_lerobot.sdk_session import run_session_cli

    run_session_cli(sys.argv[1:])


if __name__ == "__main__":
    main()
