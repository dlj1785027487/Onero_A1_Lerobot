"""遥操类型（全量 absolute / 增量 relative）的离线回归用例。

不连接 CAN、不启动 SDK 服务，也不依赖 LeRobot：

* ``teleop.reference`` 必须能一处切换所有 mapping，并在自研 CLI 与原生 CLI
  两条加载路径上同样生效；
* 未设置时保持每个 mapping 自己的声明，混合配置不能被悄悄改写；
* 全量映射的数学与它的两个前提（scale=1、启动时从臂已在保护阈值内）；
* ``bootstrap_relative_follow`` 与相对映射的组合必须在加载期就被拒绝，
  而不是等到第一次控制周期才抛 ``Relative mapping requires prepare()``。
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import read_config
from onero_a1_lerobot.arm_mapping import ArmMapper, JointSlewLimiter
from onero_a1_lerobot.sdk_config import OneroSdkConfig, OneroSdkTeleopConfig
from onero_a1_lerobot.sdk_device_config import ArmMappingConfig, SdkArmConfig
from onero_a1_lerobot.sdk_session import load_session_config

SESSION_CONFIGS_WITH_SWITCH = (
    "sdk_single_mirror.json",
    "sdk_left_leader_right_follow.json",
    "sdk_right_leader_left_follow.json",
    "sdk_dual_joint_follow.json",
    "sdk_dual_joint_follow_cameras.json",
)

JOINT_SIGNS = (1, 1, 1, 1, 1, 1, 1)


def _teleop_config(reference, mapping_references, *, mode="joint"):
    """A two-leader session with one mapping per side."""
    arms = {
        side: SdkArmConfig(model="A1L" if side == "left" else "A1R", backend="mock")
        for side in mapping_references
    }
    mappings = {
        side: ArmMappingConfig(
            source=side,
            mode=mode,
            reference=value,
            joint_signs=JOINT_SIGNS if mode == "joint" else None,
        )
        for side, value in mapping_references.items()
    }
    config = OneroSdkTeleopConfig(arms=arms, mappings=mappings, reference=reference)
    config.validate()
    return config


# --------------------------------------------------------------------------- #
# 开关：一处生效，两条加载路径都必须认
# --------------------------------------------------------------------------- #


def test_session_configs_declare_the_type_once(config_dir) -> None:
    """5 份会话配置都只在 teleop.reference 声明类型，不再逐臂声明。

    这里只钉声明位置，不钉取值：现场要试全量时改的就是这个字段，
    用例不该因为改了它而失败。
    """
    for name in SESSION_CONFIGS_WITH_SWITCH:
        raw = read_config(config_dir, name)
        assert raw["teleop"]["reference"] in ("relative", "absolute"), name
        for side, mapping in raw["teleop"]["mappings"].items():
            assert "reference" not in mapping, f"{name}: {side} 仍在逐臂声明 reference"


def test_session_configs_load_with_the_declared_type(config_dir) -> None:
    for name in SESSION_CONFIGS_WITH_SWITCH:
        raw = read_config(config_dir, name)
        declared = raw["teleop"]["reference"]
        config = load_session_config(config_dir / name)
        assert config.teleop.reference == declared, name
        for side, mapping in config.teleop.mappings.items():
            assert mapping.reference == declared, f"{name}: {side}"


def test_switch_overrides_every_mapping_without_touching_the_config(config_dir, tmp_path) -> None:
    """同一份配置只改 teleop.reference 一个字段就能切成全量。"""
    raw = read_config(config_dir, "sdk_dual_joint_follow.json")
    raw["teleop"]["reference"] = "absolute"
    probe = tmp_path / "absolute.json"
    probe.write_text(json.dumps(raw), encoding="utf-8")

    config = load_session_config(probe)
    assert {side: m.reference for side, m in config.teleop.mappings.items()} == {
        "left": "absolute",
        "right": "absolute",
    }


def test_switch_is_honoured_without_load_session_config() -> None:
    """原生 CLI 只调用 validate()，开关必须在那一层就生效。"""
    config = _teleop_config("absolute", {"left": "relative", "right": "relative"})
    assert {side: m.reference for side, m in config.mappings.items()} == {
        "left": "absolute",
        "right": "absolute",
    }

    # 反向同理：把全量配置切回增量。
    back = _teleop_config("relative", {"left": "absolute", "right": "absolute"})
    assert {side: m.reference for side, m in back.mappings.items()} == {
        "left": "relative",
        "right": "relative",
    }


def test_omitting_the_switch_keeps_a_mixed_session_mixed() -> None:
    config = _teleop_config(None, {"left": "relative", "right": "absolute"})
    assert config.reference is None
    assert {side: m.reference for side, m in config.mappings.items()} == {
        "left": "relative",
        "right": "absolute",
    }


def test_unknown_teleop_type_is_rejected() -> None:
    with pytest.raises(ValueError, match="teleop reference"):
        _teleop_config("full", {"left": "absolute", "right": "absolute"})


def test_switch_cannot_force_an_impossible_mapping() -> None:
    """相对映射会忽略 offset，所以带 offset 的会话切到增量必须当场失败。"""
    config = OneroSdkTeleopConfig(
        arms={"left": SdkArmConfig(model="A1L", backend="mock")},
        mappings={
            "right": ArmMappingConfig(
                source="left", mode="joint", reference="absolute",
                offset=(0.1,) * 7, joint_signs=JOINT_SIGNS,
            )
        },
        reference="relative",
    )
    with pytest.raises(ValueError, match="offset"):
        config.validate()


# --------------------------------------------------------------------------- #
# 映射数学
# --------------------------------------------------------------------------- #


def test_full_range_mapping_reproduces_the_leader_pose() -> None:
    """全量：q_target = signs × scale × q_leader + offset。"""
    mapper = ArmMapper(ArmMappingConfig(
        mode="joint", reference="absolute", scale=1.0, offset=(0.1,) * 7,
        joint_signs=(-1, 1, 1, -1, 1, -1, 1),
    ))
    leader = np.linspace(0.5, 2.0, 7)
    expected = np.asarray([-1, 1, 1, -1, 1, -1, 1]) * leader + 0.1
    assert np.allclose(mapper.map(leader), expected)


def test_incremental_mapping_copies_the_movement_only() -> None:
    """增量：从臂停在起始位姿，只复现主臂的位移。"""
    mapper = ArmMapper(ArmMappingConfig(
        mode="joint", reference="relative", scale=1.0, joint_signs=JOINT_SIGNS,
    ))
    mapper.capture(np.zeros(7), np.full(7, 1.0))

    assert np.allclose(mapper.map(np.full(7, 0.3)), np.full(7, 1.3))
    # 主臂原地不动时，从臂目标就是它自己的起始位姿。
    assert np.allclose(mapper.map(np.zeros(7)), np.full(7, 1.0))


def test_incremental_mapping_without_a_reference_points_at_the_native_path() -> None:
    mapper = ArmMapper(ArmMappingConfig(
        mode="joint", reference="relative", scale=1.0, joint_signs=JOINT_SIGNS,
    ))
    with pytest.raises(RuntimeError, match="bootstrap_relative_follow"):
        mapper.map(np.zeros(7))


def test_full_range_joint_mapping_refuses_a_scaled_angle() -> None:
    """scale 乘的是主臂自己的角度，会平移零点，必须拒绝。"""
    with pytest.raises(ValueError, match="shifts the follower's zero point"):
        ArmMappingConfig(
            mode="joint", reference="absolute", scale=2.0, joint_signs=JOINT_SIGNS,
        ).__post_init__()


def test_full_range_mirror_mapping_still_accepts_a_scale() -> None:
    """镜像全量没有逐关节方向表，保持原有宽容度。"""
    ArmMappingConfig(mode="mirror", reference="absolute", scale=0.5).__post_init__()


# --------------------------------------------------------------------------- #
# 全量模式的两个前提：scale=1 之外，从臂必须先就位
# --------------------------------------------------------------------------- #


def _full_range_robot(*, follower_pose, leader_pose, **overrides):
    """An OneroSdk whose single endpoint is a stand-in follower.

    ``send_action`` is driven directly so the startup guard can be exercised
    without CAN; the gripper path is empty because the arm is a mock backend.
    """
    from onero_a1_lerobot.sdk_robot import OneroSdk

    config = OneroSdkConfig(
        arms={"right": SdkArmConfig(model="A1R", backend="mock")},
        send_action=True, max_velocity=3.0, max_acceleration=80.0,
        **overrides,
    )
    config.validate()

    sent: list[np.ndarray] = []
    device = SimpleNamespace(
        connected=True,
        config=SimpleNamespace(backend="mock", channel=None),
        command=lambda q, velocities=None: sent.append(np.asarray(q, dtype=float)),
    )
    robot = object.__new__(OneroSdk)
    robot.config = config
    robot._on_fault = lambda exc: None  # a fault test would hold the arm; not here
    robot.cameras = {}
    robot.devices = {"right": device}
    robot._limiters = {"right": JointSlewLimiter(
        follower_pose, [(-3.0, 3.0)] * 7, config.max_velocity, config.max_acceleration,
        continuous=config.continuous_follow, clip_joint_limits=config.clip_joint_limits,
    )}
    robot._bootstrap_refs = {}
    robot._last_send = None
    robot._first_send = True
    robot._fault = None
    robot._next_limit_warning = {}
    robot.last_command_velocities = {}
    robot._command_frame = 0
    robot._last_gripper = {}
    robot._cache_features()
    robot.get_joint_observation = lambda: {
        **dict(zip(robot._keys_by_side["right"], follower_pose)),
        **{key: 0.0 for key in robot._gripper_key_by_side.values()},
    }
    robot.sent = sent
    robot.action = dict(zip(robot._feature_keys, leader_pose))
    return robot


def test_full_range_start_names_the_joints_that_are_out_of_position() -> None:
    follower = np.zeros(7)
    leader = np.full(7, 0.2)  # 超过默认 0.15 rad 的首帧保护
    robot = _full_range_robot(follower_pose=follower, leader_pose=leader)

    with pytest.raises(RuntimeError) as info:
        robot.send_action(robot.action)

    message = str(info.value)
    assert "first target is too far" in message
    assert "limit 0.1500 rad" in message
    assert "J1 target=+0.2000 follower=+0.0000 delta=+0.2000" in message
    assert "raise startup_max_delta_rad" in message
    assert not robot.sent, "被拒绝的首帧不能发出任何指令"
    assert robot._fault == message, "启动保护必须锁存故障并进入保位"


def test_full_range_start_is_accepted_once_the_follower_is_in_position() -> None:
    follower = np.zeros(7)
    leader = np.full(7, 0.1)
    robot = _full_range_robot(follower_pose=follower, leader_pose=leader)

    commanded = robot.send_action(robot.action)

    assert len(robot.sent) == 1
    assert set(commanded) == set(robot._feature_keys)
    assert abs(commanded[robot._feature_keys[0]] - 0.1) < 0.15


def test_raising_the_startup_limit_allows_a_far_follower() -> None:
    """保护阈值是可调的，放开后限幅器负责把从臂拉过去。"""
    follower = np.zeros(7)
    leader = np.full(7, 0.2)
    robot = _full_range_robot(
        follower_pose=follower, leader_pose=leader, startup_max_delta_rad=1.0,
    )

    robot.send_action(robot.action)

    assert len(robot.sent) == 1
    assert np.max(np.abs(robot.sent[0])) <= 0.2 + 1e-9


def test_the_guard_is_scoped_to_the_first_command() -> None:
    """首帧之后就交给跟踪误差保护，启动阈值不再参与。"""
    follower = np.zeros(7)
    robot = _full_range_robot(follower_pose=follower, leader_pose=np.zeros(7))
    robot.send_action(robot.action)

    robot.action = dict(zip(robot._feature_keys, np.full(7, 0.4)))
    robot.send_action(robot.action)  # 超过 0.15 但已不是首帧

    assert len(robot.sent) == 2


# --------------------------------------------------------------------------- #
# bootstrap 与映射类型的组合
# --------------------------------------------------------------------------- #


def test_bootstrap_relative_follow_requires_full_range_mappings(config_dir, tmp_path) -> None:
    raw = read_config(config_dir, "sdk_dual_joint_follow.json")
    # 会话配置的类型可以让现场改成全量，所以这里显式造出"相对映射 + bootstrap"这个场景。
    raw["teleop"]["reference"] = "relative"
    raw["robot"]["bootstrap_relative_follow"] = True
    probe = tmp_path / "bootstrap.json"
    probe.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(ValueError, match="bootstrap_relative_follow requires"):
        load_session_config(probe)


def test_native_configs_keep_declaring_the_type_per_arm(config_dir) -> None:
    """原生路径不经过 load_session_config，因此必须逐臂写 absolute。"""
    for name in ("native_dual_joint_follow_teleoperate.json",
                 "native_dual_joint_follow_record.json"):
        raw = read_config(config_dir, name)
        assert "reference" not in raw["teleop"], name
        assert raw["robot"]["bootstrap_relative_follow"] is True
        for side, mapping in raw["teleop"]["mappings"].items():
            assert mapping["reference"] == "absolute", f"{name}: {side}"


def test_native_configs_are_accepted_with_bootstrap(config_dir) -> None:
    """逐臂 absolute + bootstrap 的组合必须通过加载期校验。"""
    for name in ("native_dual_joint_follow_teleoperate.json",
                 "native_dual_joint_follow_record.json"):
        raw = read_config(config_dir, name)
        config = OneroSdkTeleopConfig(
            arms={
                side: SdkArmConfig(**arm) for side, arm in raw["teleop"]["arms"].items()
            },
            mappings={
                side: ArmMappingConfig(**mapping)
                for side, mapping in raw["teleop"]["mappings"].items()
            },
            **{key: value for key, value in raw["teleop"].items()
               if key in ("damping", "isolate_can_processes")},
        )
        config.validate()
        assert config.reference is None
        assert all(m.reference == "absolute" for m in config.mappings.values())
