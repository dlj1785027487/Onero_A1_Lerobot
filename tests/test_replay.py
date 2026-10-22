"""回放的离线回归用例：起始对齐的到位容差与动作列的配对。

不连接 CAN、不加载 LeRobot：

* 全量（absolute）映射要靠 ``robot.startup_max_delta_rad`` 允许限幅器把从臂
  拉到主臂位姿，所以现场会把该阈值放大（双臂配置当前是 3.0 rad）。回放的
  到位判据必须仍是自己的 0.15 rad，否则一条被挡住、停在半路的从臂会被判成
  "已对齐"并直接开始回放；
* 显式给的 ``--align-tolerance`` 只能收紧，且不得超过首帧保护阈值（改动前
  的契约不变）；
* 阈值本身比默认还紧时，到位判据跟着收紧，绝不被放宽；
* 唯一允许数据集与会话配置不一致的动作特征是每侧可选的夹爪列：``gripper_channel``
  要求真机 CAN 夹爪，所以 ``--simulate``（或 ``with_gripper=false``）的从臂没有夹爪
  特征，而真机录制的数据集有。14 维与 16 维数据集都必须能回放，其余任何名称或
  顺序差异仍然直接拒绝。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from onero_a1_lerobot.cli.replay_episode import _action_columns
from onero_a1_lerobot.features import arm_position_keys, gripper_key
from onero_a1_lerobot.replay_alignment import resolve_align_tolerance

DEFAULT_TOLERANCE_RAD = 0.15
SIDES = ("left", "right")


def _names(*, with_gripper: bool) -> list[str]:
    keys: list[str] = []
    for side in SIDES:
        keys.extend(arm_position_keys(side))
        if with_gripper:
            keys.append(gripper_key(side))
    return keys


def _robot(*, startup_max_delta_rad: float) -> SimpleNamespace:
    return SimpleNamespace(
        config=SimpleNamespace(startup_max_delta_rad=startup_max_delta_rad)
    )


# --------------------------------------------------------------------------- #
# 起始对齐的到位容差
# --------------------------------------------------------------------------- #


def test_a_widened_startup_guard_does_not_loosen_the_replay_gate() -> None:
    """双臂配置的 3.0 rad 是给启动牵引用的，回放到位判据仍是 0.15。"""
    assert resolve_align_tolerance(_robot(startup_max_delta_rad=3.0)) == pytest.approx(
        DEFAULT_TOLERANCE_RAD
    )


def test_the_configs_without_the_field_keep_their_previous_behaviour(config_dir) -> None:
    """没写该字段的配置（默认 0.15）取值与改动前完全一致。"""
    from onero_a1_lerobot.sdk_session import load_session_config

    config = load_session_config(config_dir / "sdk_single_mirror.json")
    assert config.robot.startup_max_delta_rad == pytest.approx(DEFAULT_TOLERANCE_RAD)
    robot = _robot(startup_max_delta_rad=config.robot.startup_max_delta_rad)
    assert resolve_align_tolerance(robot) == pytest.approx(DEFAULT_TOLERANCE_RAD)


def test_an_explicit_tolerance_can_only_tighten_it() -> None:
    robot = _robot(startup_max_delta_rad=3.0)

    assert resolve_align_tolerance(robot, 0.05) == pytest.approx(0.05)
    # 显式值不能绕过首帧保护阈值，这条契约与改动前一致。
    assert resolve_align_tolerance(robot, 5.0) == pytest.approx(3.0)


def test_a_guard_tighter_than_the_default_narrows_the_gate() -> None:
    assert resolve_align_tolerance(_robot(startup_max_delta_rad=0.05)) == pytest.approx(0.05)


# --------------------------------------------------------------------------- #
# 动作列配对：14 维与 16 维都要能回放
# --------------------------------------------------------------------------- #


def test_a_matching_dataset_replays_every_column() -> None:
    names = _names(with_gripper=True)
    columns, absent = _action_columns(names, names)

    assert columns == list(range(len(names)))
    assert absent == []


def test_a_sixteen_column_dataset_replays_on_gripper_less_followers() -> None:
    """--simulate 的从臂没有夹爪特征，数据集里的夹爪列被丢掉后其余照放。"""
    names = _names(with_gripper=True)
    followers = _names(with_gripper=False)
    columns, absent = _action_columns(names, followers)

    assert [names[index] for index in columns] == followers
    assert absent == []
    # 夹爪列确实被排除，而不是被错当成关节列。
    assert gripper_key("left") not in [names[index] for index in columns]


def test_a_fourteen_column_dataset_drives_a_gripper_equipped_follower() -> None:
    """反过来：数据集没录夹爪时，从臂保持当前开合度。"""
    names = _names(with_gripper=False)
    followers = _names(with_gripper=True)
    columns, absent = _action_columns(names, followers)

    assert columns == list(range(len(names)))
    assert absent == [gripper_key("left"), gripper_key("right")]


def test_only_the_optional_gripper_column_may_differ() -> None:
    names = _names(with_gripper=True)
    followers = _names(with_gripper=False)
    # 数据集少了一个关节列（不是夹爪列）时仍然必须拒绝。
    with pytest.raises(SystemExit, match="do not record"):
        _action_columns([name for name in names if name != "left_arm.joint3-l.pos"], followers)

    # 多出一个非夹爪列同样必须拒绝。
    with pytest.raises(SystemExit, match="no matching follower feature"):
        _action_columns([*names, "torso.joint1.pos"], followers)


def test_a_reordered_dataset_is_still_rejected() -> None:
    names = _names(with_gripper=False)
    swapped = [names[1], names[0], *names[2:]]

    with pytest.raises(SystemExit, match="names/order do not match"):
        _action_columns(swapped, names)


def test_unnamed_dataset_features_are_rejected() -> None:
    with pytest.raises(SystemExit, match="unnamed"):
        _action_columns(None, _names(with_gripper=True))
