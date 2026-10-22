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
    parser.add_argument("--align-tolerance", type=float, default=None, help="Alignment feedback tolerance in radians (default: 0.15; never looser than robot.startup_max_delta_rad)")
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


# The optional per-side gripper column is the only action feature a dataset and a
# session config may disagree on. ``gripper_channel`` needs a real CAN gripper, so
# a ``--simulate`` run -- and any config with ``with_gripper=false`` -- has no
# gripper feature at all, while the datasets this project records on hardware do.
# Recorded data is replayed with that column dropped, and a dataset without it
# drives a gripper-equipped follower with the opening it already holds. Every
# other name, and the order, must still match exactly.
_GRIPPER_FEATURE_SUFFIX = ".gripper.pos"


def _action_columns(dataset_names, robot_keys) -> tuple[list[int], list[str]]:
    """Pair recorded action columns with the configured follower features.

    Returns the dataset column indices to read for each frame, in follower order,
    plus the follower gripper keys the dataset does not record.
    """
    dataset_names = [str(name) for name in dataset_names or ()]
    robot_keys = [str(key) for key in robot_keys]
    if not dataset_names:
        raise SystemExit(
            "Dataset action features are unnamed, so they cannot be matched against "
            "the configured SDK followers"
        )
    recorded = set(dataset_names)
    absent = [key for key in robot_keys if key not in recorded]
    for key in absent:
        if not key.endswith(_GRIPPER_FEATURE_SUFFIX):
            raise SystemExit(
                f"Dataset action columns do not record {key}, which the configured "
                "followers require; only the optional gripper column may be absent"
            )
    kept = [name for name in dataset_names if name in set(robot_keys)]
    if kept != [key for key in robot_keys if key not in set(absent)]:
        raise SystemExit(
            "Dataset action names/order do not match the configured SDK followers: "
            f"dataset={dataset_names}, followers={robot_keys}"
        )
    for name in (name for name in dataset_names if name not in set(robot_keys)):
        if not name.endswith(_GRIPPER_FEATURE_SUFFIX):
            raise SystemExit(
                f"Dataset action column {name} has no matching follower feature; only "
                "the optional gripper column may differ"
            )
    columns = [index for index, name in enumerate(dataset_names) if name in set(robot_keys)]
    return columns, absent


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
    action_names = list(dataset.features[ACTION].get("names") or ())
    columns, held_gripper_keys = _action_columns(action_names, robot.action_features)
    played_names = [action_names[index] for index in columns]
    if len(columns) != len(action_names):
        _logger.warning(
            "Dataset %s records %s, which the configured followers have no gripper "
            "channel for; replaying its other %d action columns",
            args.repo_id,
            ", ".join(name for name in action_names if name not in set(played_names)),
            len(played_names),
        )
    if held_gripper_keys:
        _logger.warning(
            "Dataset %s does not record %s; the follower gripper keeps the opening it "
            "already holds for the whole episode",
            args.repo_id, ", ".join(held_gripper_keys),
        )
    if dataset.num_frames <= 0:
        raise SystemExit("The selected episode has no frames to replay")
    # Read action columns only: decoding both recorded videos inside the CAN
    # loop adds latency even though playback never consumes camera frames.
    actions = dataset.select_columns([ACTION])

    # Materialize and validate frame zero before any arm is enabled. Reuse it
    # for both alignment and playback, without reading live camera images.
    def frame_action(values, index: int) -> dict[str, float]:
        return _frame_action([values[column] for column in columns], played_names, index)

    first_action = frame_action(actions[0][ACTION], 0)
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
        held_opening: dict[str, float] = {}
        if held_gripper_keys:
            # Latched before aligning: the alignment target must contain every
            # configured follower feature.
            observed = robot.get_joint_observation()
            held_opening = {key: float(observed[key]) for key in held_gripper_keys}
            first_action = {**first_action, **held_opening}
        if not args.no_align:
            align_to_first_frame(
                robot, first_action, minimum_duration_s=args.align_duration,
                max_velocity=args.align_max_velocity, max_acceleration=args.align_max_acceleration,
                settle_timeout_s=args.align_timeout, joint_tolerance_rad=args.align_tolerance,
            )
        for i in range(dataset.num_frames):
            start = time.perf_counter()
            action = first_action if i == 0 else {
                **frame_action(actions[i][ACTION], i), **held_opening,
            }
            robot.send_action(action)
            if i % max(1, int(fps)) == 0 or i == dataset.num_frames - 1:
                _logger.info("Replayed frame %d/%d", i + 1, dataset.num_frames)
            sleep_s = period - (time.perf_counter() - start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    _logger.info("Replay finished: %d frames", dataset.num_frames)


if __name__ == "__main__":
    main()
