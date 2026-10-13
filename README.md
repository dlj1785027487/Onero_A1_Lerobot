# Onero A1 LeRobot

[中文说明](README_zh.md) · [Detailed usage](docs/usage_sdk_follow.md) · [Cameras and data recovery](docs/usage_usb_cameras.md)

A1 single/dual-arm teleoperation and data collection over CAN/SLCAN, with USB cameras, LeRobot datasets and action replay.

## Install

Use Linux and Python 3.12+. Place `armsdk` beside this project, then run from the project root:

```bash
A1_LEROBOT_VENV="$PWD/.venv" bash scripts/setup.sh
source .venv/bin/activate
```

Set `A1_ARM_SDK_PATH` if the SDK is elsewhere. Activate the environment in each new terminal. Use setup's `--base` option for teleoperation without recording dependencies.
Exit with `deactivate` or `source .venv/bin/deactivate`; source the script rather than executing it directly.

## Configure devices

| Configuration | Purpose |
|---|---|
| `configs/sdk_left_leader_right_follow.json` | Left leader → right follower (`joint` mapping) |
| `configs/sdk_right_leader_left_follow.json` | Right leader → left follower (`joint` mapping) |
| `configs/sdk_single_mirror.json` | Left leader → right follower (`mirror` mapping) |
| `configs/sdk_dual_joint_follow.json` | Dual-arm teleoperation |
| `configs/sdk_dual_joint_follow_cameras.json` | Dual-arm teleoperation and camera recording |
| `configs/native_dual_joint_follow_teleoperate.json` | Native `lerobot-teleoperate` (`--config_path`) |
| `configs/native_dual_joint_follow_record.json` | Native `lerobot-record` (adds dataset settings) |

Update the configuration to match your wiring:

- `teleop.arms`: leaders; `robot.arms`: followers. Each physical arm needs its own channel.
- `channel`: a SocketCAN interface or SLCAN `/dev/serial/by-id/...` path. Do not run `slcand` on a port used for direct SLCAN.
- `robot.cameras`: camera paths, resolution and frame rate. Prefer stable `/dev/v4l/by-path/...` paths.

See [detailed usage](docs/usage_sdk_follow.md) for CAN setup. Device paths in the examples must be adapted to the deployment computer.

## Teleoperate

```bash
onero-a1-lerobot-teleoperate \
  --session-config configs/sdk_dual_joint_follow.json
```

Add `--simulate --duration 5` for a mock session without opening arms or USB cameras.

The same dual-arm following and recording also run under LeRobot's own CLIs:

```bash
lerobot-teleoperate --config_path=configs/native_dual_joint_follow_teleoperate.json
lerobot-record --config_path=configs/native_dual_joint_follow_record.json
```

The native path never calls `prepare()`, so it bootstraps the relative reference from the first
action through `robot.bootstrap_relative_follow`. The followers move exactly as they do under the
project CLI, but the loop runs at `dataset.fps` (30 Hz here) and the dataset `action` column has a
different meaning; see section 5 of the [detailed usage](docs/usage_sdk_follow.md).

## Record

```bash
onero-a1-lerobot-record \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --task "dual-arm manipulation" \
  --root ./datasets/local/a1_cameras
```

Recording continues until **Ctrl+C stops and saves it**. Add `--duration 60` to limit the session to 60 seconds; asynchronous recording preparation counts toward that time.
`--root` is the exact dataset directory. Use a configuration without cameras to record joints and actions only.

Interactive commands require Enter:

| Command | Action |
|---|---|
| `r` | Stop recording and save; press again after saving to start another episode |
| `p` | Pause/resume following; pausing ends the current recording |
| `q` | Exit the session and save |

## Replay actions

```bash
onero-a1-replay-episode \
  --session-config configs/sdk_dual_joint_follow_cameras.json \
  --repo-id local/a1_cameras --root ./datasets/local/a1_cameras --episode 0
```

Replay smoothly aligns the followers to frame zero before playing actions (at least 3 seconds; use `--align-duration 5` for a longer transition).
Alignment does not plan around obstacles; clear the motion path first. Cameras stay closed. `--no-align` skips alignment but retains the initial-distance guard; `--simulate` enables mock replay.

## Notes

- Hardware sessions enable the arms. Exiting disables them; support the arms before exit.
- Capture saves source images and samples; **videos and datasets are generated after recording stops**. Wait for `Episode writer finished ... status=saved` before closing the terminal. Start the next episode after saving finishes.
- Appending requires compatible camera, joint, frame-rate and encoder settings. Use a new dataset directory when changing them.
- Failed conversion retains the capture source. See the [camera guide](docs/usage_usb_cameras.md) for recovery commands, encoding options and timeouts.
- CAN arms with `with_gripper=true` include gripper control and recording (0–1, where 1 is open); mock/MuJoCo sessions do not. The physical protocol is selected explicitly by `gripper_protocol` (`onero` default, or `omnipicker`); `none` sends no gripper CAN frames. `gripper_model` (`woan_picker` default) independently selects the visual Link7 tool model. Both are follower-only settings; the leader keeps the fixed `onero`/`woan_picker` squeeze-handle source. See [detailed usage](docs/usage_sdk_follow.md) for simulation, visualization and policy interfaces.
- Hardware-free cases cover configuration and video encoding: run `python -m pytest` inside the venv. Before a real run, `python scripts/can_precheck.py` reads every motor state without enabling or moving anything. See section 6 of the [detailed usage](docs/usage_sdk_follow.md).
