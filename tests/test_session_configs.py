"""所有会话配置必须在无 CAN / 无相机时也能加载并校验通过。"""

from __future__ import annotations

from conftest import read_config

SESSION_CONFIGS = (
    "sdk_single_mirror.json",
    "sdk_left_leader_right_follow.json",
    "sdk_right_leader_left_follow.json",
    "sdk_dual_joint_follow.json",
    "sdk_dual_joint_follow_cameras.json",
)

EXPECTED_LEADER_CHANNELS = {"left": "can4", "right": "can3"}
EXPECTED_FOLLOWER_CHANNELS = {"left": "can2", "right": "can1"}


def test_session_configs_use_only_the_documented_top_level_keys(config_dir):
    for name in SESSION_CONFIGS:
        assert set(read_config(config_dir, name)) == {"robot", "teleop", "recording"}


def test_session_configs_load_and_validate_without_hardware(config_dir):
    from onero_a1_lerobot.sdk_session import load_session_config

    for name in SESSION_CONFIGS:
        config = load_session_config(config_dir / name, simulate="mock")
        assert config.recording.video_encoder == "auto"
        # 单臂配置只有一侧从臂；双臂配置两侧都有。
        assert config.robot.arms, name
        assert set(config.robot.arms) <= {"left", "right"}


def test_simulate_turns_camera_configs_into_usable_mocks(config_dir):
    """docs/usage_usb_cameras.md 明确支持 --simulate + 相机配置。"""
    from onero_a1_lerobot.sdk_session import load_session_config

    config = load_session_config(config_dir / "sdk_dual_joint_follow_cameras.json", simulate="mock")
    assert set(config.robot.cameras) == {"left_wrist", "right_wrist"}
    for camera in config.robot.cameras.values():
        assert camera.backend == "mock"
        assert camera.device is None
        # V4L2 曝光控制在合成图像上没有意义，mock 相机会拒绝它们。
        assert camera.auto_exposure is None
        assert camera.exposure_absolute is None
        assert (camera.width, camera.height, camera.fps) == (1600, 1200, 30)


def test_dual_arm_configs_drive_the_deployed_can_wiring(config_dir):
    """双臂配置的通道必须与现场接线一致（见 docs/usage_sdk_follow.md）。"""
    for name in ("sdk_dual_joint_follow.json", "sdk_dual_joint_follow_cameras.json"):
        config = read_config(config_dir, name)
        leaders = {side: arm["channel"] for side, arm in config["teleop"]["arms"].items()}
        followers = {side: arm["channel"] for side, arm in config["robot"]["arms"].items()}
        assert leaders == EXPECTED_LEADER_CHANNELS
        assert followers == EXPECTED_FOLLOWER_CHANNELS


def test_camera_config_differs_from_the_dual_config_only_by_cameras(config_dir):
    """两份双臂会话配置除 `robot.cameras` 与显式 recording 默认值外必须一致。"""
    plain = read_config(config_dir, "sdk_dual_joint_follow.json")
    cameras = read_config(config_dir, "sdk_dual_joint_follow_cameras.json")

    plain_robot = {k: v for k, v in plain["robot"].items() if k != "cameras"}
    camera_robot = {k: v for k, v in cameras["robot"].items() if k != "cameras"}
    assert "cameras" not in plain_robot and "cameras" not in camera_robot
    assert plain_robot == camera_robot
    assert plain["teleop"] == cameras["teleop"]

    # recording 段只允许出现显式写出的默认值差异。
    recording_diff = {
        key for key in set(plain["recording"]) | set(cameras["recording"])
        if plain["recording"].get(key) != cameras["recording"].get(key)
    }
    assert recording_diff <= {"max_queue_bytes", "video_encoder"}


def test_camera_config_declares_two_wrist_cameras(config_dir):
    config = read_config(config_dir, "sdk_dual_joint_follow_cameras.json")
    cameras = config["robot"]["cameras"]
    assert set(cameras) == {"left_wrist", "right_wrist"}
    for camera in cameras.values():
        assert camera["backend"] == "usb"
        assert (camera["width"], camera["height"], camera["fps"]) == (1600, 1200, 30)


def test_session_recording_section_rejects_an_unknown_encoder():
    import pytest

    from onero_a1_lerobot.episode_recording import RecordingConfig

    RecordingConfig().validate()
    with pytest.raises(ValueError, match="video_encoder"):
        RecordingConfig(video_encoder="av1").validate()
