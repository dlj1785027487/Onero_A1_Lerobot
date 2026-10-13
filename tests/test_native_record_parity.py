"""原生 ``lerobot-record`` 配置必须与自研录制管线保持一致。

背景（对比 ``datasets/cmp_native`` 与 ``datasets/cmp_session`` 得到）：

- 原生 ``lerobot-record`` 不写 ``dataset.rgb_encoder`` 时回落到 LeRobot 默认的
  ``libsvtav1``（``video.codec=av1``、GOP=2、crf=30、preset=12）；
- 自研管线固定 ``h264``（GOP=帧率、crf=23、preset=ultrafast、
  ``tune=zerolatency``、``bf=0``）。

两份配置的机械臂接线、相机和编码参数必须逐项相同；只有已写入
``docs/usage_sdk_follow.md`` 的四处结构性差异允许保留：``type`` 字段、遥操类型的声明位置
（自研路径用会话级 ``teleop.reference``，原生路径逐臂写 ``reference``）、
``robot.bootstrap_relative_follow``，以及录制配置的 ``dataset`` 段。
"""

from __future__ import annotations

import pytest
from conftest import read_config

NATIVE_RECORD = "native_dual_joint_follow_record.json"
NATIVE_TELEOP = "native_dual_joint_follow_teleoperate.json"
SESSION_CAMERAS = "sdk_dual_joint_follow_cameras.json"

ARM_KEYS = (
    "model", "backend", "channel", "mounting",
    "with_gripper", "gripper_model", "gripper_protocol",
    "handle_open_code", "handle_closed_code",
)


def _parse_native_record(config_dir):
    draccus = pytest.importorskip("draccus")
    pytest.importorskip("lerobot")
    from lerobot.scripts.lerobot_record import RecordConfig

    import lerobot_robot_onero_a1  # noqa: F401  注册 onero_sdk / onero_sdk_teleop

    # Pass an explicit empty argv: draccus otherwise re-parses pytest's own options.
    return draccus.parse(
        RecordConfig, args=[], config_path=str(config_dir / NATIVE_RECORD),
    )


def test_native_record_config_is_accepted_by_lerobot(config_dir):
    """整份配置（含两路相机与 dataset 段）必须能被原生 CLI 解析。"""
    native = _parse_native_record(config_dir)
    assert native.dataset.fps == 30
    assert set(native.robot.cameras) == {"left_wrist", "right_wrist"}
    assert native.robot.send_action is True


def test_native_record_video_encoder_matches_session_pipeline(config_dir):
    """``dataset.rgb_encoder`` 必须等于自研管线为同一 fps 生成的编码设置。"""
    from onero_a1_lerobot.episode_recording import RecordingConfig, build_rgb_encoder
    from onero_a1_lerobot.ffmpeg_encoder import resolve_video_backend

    native = _parse_native_record(config_dir)
    expected = build_rgb_encoder(
        RecordingConfig(fps=native.dataset.fps), resolve_video_backend("libx264"),
    )
    actual = native.dataset.rgb_encoder

    assert actual.vcodec == expected.vcodec == "h264"
    assert actual.pix_fmt == expected.pix_fmt == "yuv420p"
    assert actual.g == expected.g == native.dataset.fps
    assert actual.crf == expected.crf == 23
    assert actual.preset == expected.preset == "ultrafast"
    assert actual.extra_options == expected.extra_options == {"tune": "zerolatency", "bf": "0"}
    assert actual.get_codec_options(1, as_strings=True) == expected.get_codec_options(1, as_strings=True)


def test_native_and_session_configs_share_robot_wiring(config_dir):
    """两份配置必须驱动同一批机械臂、夹爪和相机。"""
    native = read_config(config_dir, NATIVE_RECORD)
    session = read_config(config_dir, SESSION_CAMERAS)

    assert native["robot"]["cameras"] == session["robot"]["cameras"]
    for section in ("robot", "teleop"):
        native_arms, session_arms = native[section]["arms"], session[section]["arms"]
        assert set(native_arms) == set(session_arms) == {"left", "right"}
        for side, arm in native_arms.items():
            for key in ARM_KEYS:
                assert arm.get(key) == session_arms[side].get(key), f"{section}.{side}.{key}"


def test_native_configs_keep_only_the_documented_differences(config_dir):
    """原生路径的四处结构性差异，改动时必须同步文档。"""
    native = read_config(config_dir, NATIVE_RECORD)
    teleop = read_config(config_dir, NATIVE_TELEOP)
    session = read_config(config_dir, SESSION_CAMERAS)

    for config in (native, teleop):
        assert config["robot"]["type"] == "onero_sdk"
        assert config["teleop"]["type"] == "onero_sdk_teleop"
        assert config["robot"]["bootstrap_relative_follow"] is True
        assert config["teleop"]["mappings"]["left"]["reference"] == "absolute"
        assert config["teleop"]["mappings"]["right"]["reference"] == "absolute"
        # 原生路径逐臂声明遥操类型，所以顶层不带那个会话级开关。
        assert "reference" not in config["teleop"]

    # 自研路径反过来：一枚会话级开关，映射里不再重复声明。
    assert session["teleop"]["reference"] == "relative"
    for mapping in session["teleop"]["mappings"].values():
        assert "reference" not in mapping

    # 原生 teleoperate 配置不含 dataset；原生 record 配置必须含 dataset 段。
    assert "dataset" not in teleop
    assert "dataset" in native
    assert set(teleop["robot"]["cameras"]) == {"left_wrist", "right_wrist"}
