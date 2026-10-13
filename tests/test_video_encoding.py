"""编码产出验证：用原生录制配置里的 ``rgb_encoder`` 真编一段 MP4 再回读校验。

``tests/test_native_record_parity.py`` 只比较配置字段；这里把同一组字段交给自研
编码器实际编码，证明落盘产物的 codec / 像素格式 / GOP 与配置一致（对比数据集中
``cmp_native`` 落的是 av1、``cmp_session`` 落的是 h264，正是本用例防回归的点）。
"""

from __future__ import annotations

import json

import pytest

numpy = pytest.importorskip("numpy")
pytest.importorskip("av")
pytest.importorskip("lerobot")

NATIVE_RECORD = "native_dual_joint_follow_record.json"
FPS = 30
HEIGHT = 64
WIDTH = 96
FRAMES = 5


@pytest.fixture(scope="module")
def native_rgb_encoder(config_dir):
    from lerobot.configs.video import RGBEncoderConfig

    raw = json.loads((config_dir / NATIVE_RECORD).read_text(encoding="utf-8"))
    return RGBEncoderConfig(**raw["dataset"]["rgb_encoder"])


def test_native_encoder_writes_h264_mp4(tmp_path, native_rgb_encoder):
    import av

    from onero_a1_lerobot.streaming_recording import SynchronousVideoEncoder

    key = "observation.images.left_wrist"
    encoder = SynchronousVideoEncoder(fps=FPS, rgb_encoder=native_rgb_encoder, encoder_threads=1)
    encoder.prepare_episode([key], tmp_path, {key: (HEIGHT, WIDTH, 3)})
    rng = numpy.random.default_rng(0)
    for _ in range(FRAMES):
        encoder.feed_frame(key, rng.integers(0, 256, (HEIGHT, WIDTH, 3), dtype=numpy.uint8))
    # seal_episode re-probes the finished file and rejects a wrong codec/size/fps/count.
    encoder.seal_episode(expected_frames=FRAMES)
    results = encoder.finish_episode()

    path, stats = results[key]
    assert path.exists()
    # ``count`` counts one sample per pixel per fed frame (all three channels).
    assert stats["count"][0] == FRAMES * HEIGHT * WIDTH
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        assert stream.codec.canonical_name == "h264"
        assert stream.pix_fmt == "yuv420p"
        assert (stream.height, stream.width) == (HEIGHT, WIDTH)
        assert stream.frames == FRAMES
        assert stream.base_rate == FPS


def test_encoder_rejects_frames_without_a_started_episode(tmp_path, native_rgb_encoder):
    from onero_a1_lerobot.streaming_recording import SynchronousVideoEncoder

    encoder = SynchronousVideoEncoder(fps=FPS, rgb_encoder=native_rgb_encoder, encoder_threads=1)
    with pytest.raises(RuntimeError, match="Start a streaming episode"):
        encoder.feed_frame("observation.images.left_wrist", numpy.zeros((HEIGHT, WIDTH, 3), numpy.uint8))
    encoder.close()


def test_resolve_video_backend_rejects_unknown_encoder():
    from onero_a1_lerobot.ffmpeg_encoder import resolve_video_backend

    with pytest.raises(ValueError, match="video_encoder"):
        resolve_video_backend("av1")
    assert resolve_video_backend("libx264").name == "libx264"
