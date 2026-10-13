"""Replay a LeRobot episode on A1 SDK follower arms."""

from __future__ import annotations

import argparse
import logging
import math
import time

_logger = logging.getLogger("onero_a1.replay")


def _import_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.utils.constants import ACTION
    except Exception as exc:
        raise RuntimeError("LeRobotDataset is not available. Install LeRobot first.") from exc
    return LeRobotDataset, ACTION


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a LeRobot episode on A1 SDK follower arms.")
    parser.add_argument("--session-config", required=True, help="SDK session JSON; only follower arms connect")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. local/a1_follow")
    parser.add_argument("--root", default=None, help="Exact local dataset directory")
    parser.add_argument("--episode", type=int, default=0, help="Episode index to replay")
    parser.add_argument("--fps", type=int, default=None, help="Replay frame rate (default: dataset fps)")
    parser.add_argument("--simulate", action="store_true", help="Use mock followers without opening CAN")
    parser.add_argument("--no-align", action="store_true", help="Skip automatic start-pose alignment; initial-distance protection still applies")
    parser.add_argument("--align-duration", type=float, default=3.0, help="Minimum smooth start-pose transition time in seconds (default: 3)")
    parser.add_argument("--align-max-velocity", type=float, default=0.2, help="Alignment joint velocity cap in rad/s (default: 0.2)")
    parser.add_argument("--align-max-acceleration", type=float, default=0.5, help="Alignment joint acceleration cap in rad/s^2 (default: 0.5)")
    parser.add_argument("--align-timeout", type=float, default=15.0, help="Extra time in seconds to settle after the planned transition (default: 15)")
    parser.add_argument("--align-tolerance", type=float, default=None, help="Alignment feedback tolerance in radians (defaults to robot.startup_max_delta_rad; cannot exceed it)")
    args = parser.parse_args()
    if args.episode < 0:
        parser.error("--episode must be nonnegative")
    if args.fps is not None and args.fps <= 0:
        parser.error("--fps must be positive")
    for name in ("align_duration", "align_max_velocity", "align_max_acceleration", "align_timeout", "align_tolerance"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    return args


def _frame_action(values, names, index: int) -> dict[str, float]:
    if len(values) != len(names):
        raise ValueError(f"Replay frame {index} action length does not match its feature names")
    action = {name: float(values[j]) for j, name in enumerate(names)}
    for name, value in action.items():
        if not math.isfinite(value):
            raise ValueError(f"Replay frame {index} contains a non-finite {name}")
        if name.endswith(".gripper.pos") and not 0.0 <= value <= 1.0:
            raise ValueError(f"Replay frame {index} gripper target {name} is outside [0, 1]")
    return action


def main() -> None:
    from onero_a1_lerobot.replay_alignment import align_to_first_frame
    from onero_a1_lerobot.sdk_robot import OneroSdk
    from onero_a1_lerobot.sdk_session import load_session_config

    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        force=True,
    )
    session_config = load_session_config(args.session_config, simulate=args.simulate)
    # Playback consumes recorded actions and never needs live USB images.
    session_config.robot.cameras = {}
    if not session_config.robot.send_action:
        raise SystemExit("SDK replay requires robot.send_action=true in the session config")
    robot = OneroSdk(session_config.robot)
    try:
        LeRobotDataset, ACTION = _import_lerobot_dataset()
    except RuntimeError as exc:
        raise SystemExit(
            f"ERROR: {exc}\nInstall LeRobot support with: python3 -m pip install -e '.[lerobot]'"
        ) from None
    dataset = LeRobotDataset(
        args.repo_id, root=args.root, episodes=[args.episode], video_backend="pyav",
        download_videos=False,
    )
    if ACTION not in dataset.features:
        raise SystemExit(f"ERROR: Dataset {args.repo_id} does not contain action features.")
    action_names = dataset.features[ACTION].get("names")
    if tuple(action_names or ()) != tuple(robot.action_features):
        raise SystemExit("Dataset action names/order do not match the configured SDK followers")
    if dataset.num_frames <= 0:
        raise SystemExit("The selected episode has no frames to replay")
    # Read action columns only: decoding both recorded videos inside the CAN
    # loop adds latency even though playback never consumes camera frames.
    actions = dataset.select_columns([ACTION])
    # Materialize and validate frame zero before any arm is enabled. Reuse it
    # for both alignment and playback, without reading live camera images.
    first_action = _frame_action(actions[0][ACTION], action_names, 0)
    fps = args.fps if args.fps is not None else dataset.fps
    if not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
        raise SystemExit("Dataset fps must be finite and positive")
    period = 1.0 / fps
    _logger.info(
        "Replaying: repo=%s episode=%d frames=%d fps=%g",
        args.repo_id, args.episode, dataset.num_frames, fps,
    )
    _logger.info("Connecting enables follower arms; exiting disables them")
    with robot:
        if not args.no_align:
            align_to_first_frame(
                robot, first_action, minimum_duration_s=args.align_duration,
                max_velocity=args.align_max_velocity, max_acceleration=args.align_max_acceleration,
                settle_timeout_s=args.align_timeout, joint_tolerance_rad=args.align_tolerance,
            )
        for i in range(dataset.num_frames):
            start = time.perf_counter()
            action = first_action if i == 0 else _frame_action(actions[i][ACTION], action_names, i)
            robot.send_action(action)
            if i % max(1, int(fps)) == 0 or i == dataset.num_frames - 1:
                _logger.info("Replayed frame %d/%d", i + 1, dataset.num_frames)
            sleep_s = period - (time.perf_counter() - start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    _logger.info("Replay finished: %d frames", dataset.num_frames)


if __name__ == "__main__":
    main()
