"""Print A1 SDK follower feedback while keeping the connected arms in hold."""

from __future__ import annotations

import argparse
import logging
import math
import time
from pprint import pprint

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print A1 SDK follower joint feedback. Connecting enables the follower arms "
            "and continuously holds their positions; exiting disables them."
        )
    )
    parser.add_argument("--session-config", required=True, help="SDK session JSON; only follower arms connect")
    parser.add_argument("--simulate", action="store_true", help="Use mock followers without opening CAN")
    parser.add_argument("--rate", type=float, default=1.0, help="Print rate in Hz (up to robot.control_hz)")
    parser.add_argument("--count", type=int, default=0, help="Samples to print; 0 means until Ctrl+C")
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")
    if args.count < 0:
        parser.error("--count must be nonnegative")
    return args


def main() -> None:
    from onero_a1_lerobot.sdk_robot import OneroSdk
    from onero_a1_lerobot.sdk_session import load_session_config

    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    config = load_session_config(args.session_config, simulate=args.simulate).robot
    if not config.send_action:
        raise SystemExit("Observation holding requires robot.send_action=true in the session config")
    if args.rate > config.control_hz:
        raise SystemExit("--rate must not exceed robot.control_hz")
    logging.info("Connecting enables follower arms and holds their positions; exiting disables them")
    robot = OneroSdk(config)
    try:
        with robot:
            printed = 0
            next_print = time.monotonic()
            period = 1.0 / config.control_hz
            while args.count == 0 or printed < args.count:
                start = time.monotonic()
                robot.hold()
                observation = robot.get_observation()
                if start >= next_print:
                    pprint({
                        key: {"shape": value.shape, "dtype": str(value.dtype)}
                        if isinstance(value, np.ndarray) else value
                        for key, value in observation.items()
                    })
                    printed += 1
                    next_print = start + 1.0 / args.rate
                if args.count and printed >= args.count:
                    break
                sleep_s = period - (time.monotonic() - start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
