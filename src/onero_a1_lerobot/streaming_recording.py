"""Encode RGB groups directly to LeRobot videos in the recording process.

The fixed LeRobot dataset factory calls this encoder through its streaming
interface. Camera tasks run in parallel within a group. The final feed call
waits until every task consumes its shared RGB view, by software encoding or
copying to a bounded FFmpeg pipe. No queue silently drops image frames.
"""

from __future__ import annotations

import logging
import math
import shutil
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

from .ffmpeg_encoder import FfmpegCameraEncoder, VideoBackend

logger = logging.getLogger(__name__)


class _RgbHistogramStats:
    """Exact statistics over sampled uint8 pixels, with fixed-size histograms."""

    def __init__(self, np):
        self._np = np
        self._histograms = np.zeros((3, 256), dtype=np.int64)
        self._count = 0

    def update(self, pixels) -> None:
        for channel in range(3):
            self._histograms[channel] += self._np.bincount(pixels[:, channel], minlength=256)
        self._count += pixels.shape[0]

    def get_statistics(self) -> dict:
        if self._count == 0:
            raise ValueError("Cannot compute RGB statistics without pixels")
        np = self._np
        values = np.arange(256, dtype=np.float64)
        mean = (self._histograms * values).sum(axis=1) / self._count
        variance = (self._histograms * (values - mean[:, None]) ** 2).sum(axis=1) / self._count
        populated = self._histograms != 0
        stats = {
            "min": populated.argmax(axis=1).astype(np.float64),
            "max": (255 - populated[:, ::-1].argmax(axis=1)).astype(np.float64),
            "mean": mean,
            "std": np.sqrt(variance),
            "count": np.asarray([self._count], dtype=np.int64),
        }
        cumulative = self._histograms.cumsum(axis=1)
        for quantile in (0.01, 0.10, 0.50, 0.90, 0.99):
            rank = quantile * (self._count - 1)
            lower, upper = math.floor(rank), math.ceil(rank)
            low_values = np.asarray([
                np.searchsorted(cumulative[channel], lower, side="right") for channel in range(3)
            ], dtype=np.float64)
            high_values = np.asarray([
                np.searchsorted(cumulative[channel], upper, side="right") for channel in range(3)
            ], dtype=np.float64)
            stats[f"q{int(quantile * 100):02d}"] = low_values + (high_values - low_values) * (rank - lower)
        return stats


@dataclass
class _CameraStream:
    name: str
    directory: Path
    path: Path
    stats: Any
    shape: tuple[int, int, int] | None = None
    container: Any = None
    stream: Any = None
    external: FfmpegCameraEncoder | None = None
    frames: int = 0
    last_encode_s: float = 0.0
    last_stats_s: float = 0.0


class SynchronousVideoEncoder:
    """LeRobot streaming interface without asynchronous RGB buffering.

    The recording process calls the public methods serially. Each camera's
    container and statistics belong to at most one encoder task at a time.
    PyAV, NumPy, and LeRobot are imported only when this object is constructed.
    """

    def __init__(self, fps, rgb_encoder=None, depth_encoder=None, encoder_threads=None, backend=None):
        import av
        import numpy as np
        from lerobot.configs.video import rgb_encoder_defaults

        if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
            raise ValueError("Streaming recording fps must be a positive integer")
        if encoder_threads is not None and (
            isinstance(encoder_threads, bool)
            or not isinstance(encoder_threads, int) or encoder_threads <= 0
        ):
            raise ValueError("Encoder threads must be a positive integer or None")
        self.fps = fps
        self._av = av
        self._np = np
        self._rgb_encoder = rgb_encoder if rgb_encoder is not None else rgb_encoder_defaults()
        self._encoder_threads = encoder_threads
        self._backend = backend or VideoBackend("libx264")
        self._time_base = Fraction(1, fps)
        self._streams: dict[str, _CameraStream] = {}
        self._expected_video_info: dict[str, dict] = {}
        self._owned_directories: set[Path] = set()
        self._keys: tuple[str, ...] = ()
        self._root: Path | None = None
        self._executor: ThreadPoolExecutor | None = None
        self._pending: dict[str, Any] = {}
        self._groups = 0
        self._poison: str | None = None
        self._active = False
        self._sealed = False
        self._closed = False
        self.last_group_s = 0.0

    @property
    def frame_counts(self) -> dict[str, int]:
        return {name: stream.frames for name, stream in self._streams.items()}

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("Streaming recorder is closed")
        if self._poison is not None:
            raise RuntimeError(f"Recording episode is incomplete: {self._poison}")

    def _fail(self, message: str) -> None:
        if self._poison is None:
            self._poison = message

    def start_episode(self, video_keys, temp_dir, depth_video_keys=None) -> None:
        self._check_open()
        if depth_video_keys:
            raise ValueError("A1 streaming recorder supports RGB cameras only")
        keys = tuple(video_keys)
        root = Path(temp_dir)
        if not keys or len(set(keys)) != len(keys):
            raise ValueError("Streaming recording requires distinct camera keys")
        if any(not isinstance(key, str) or not key for key in keys):
            raise ValueError("Streaming camera keys must be nonempty strings")
        if self._active:
            # DatasetWriter calls start_episode again on its first add_frame,
            # after prepare_episode has already opened this episode's codecs.
            if keys == self._keys and root == self._root and self._groups == 0 and not self._pending:
                return
            raise RuntimeError("Finish or cancel the active streaming episode before starting another")
        self._keys, self._root = keys, root
        self._groups = 0
        self._active = True
        self._sealed = False
        try:
            for key in keys:
                directory = Path(tempfile.mkdtemp(prefix="a1-stream-", dir=root))
                self._owned_directories.add(directory)
                self._streams[key] = _CameraStream(
                    key, directory, directory / "stream.mp4", _RgbHistogramStats(self._np),
                )
            if len(keys) > 1:
                self._executor = ThreadPoolExecutor(
                    max_workers=len(keys), thread_name_prefix="a1-video-encode",
                )
        except BaseException:
            try:
                self.cancel_episode()
            except Exception:
                logger.exception("Could not fully clean up streaming startup")
            raise

    @staticmethod
    def _validate_shape(shape) -> tuple[int, int, int]:
        if (
            not isinstance(shape, (tuple, list)) or len(shape) != 3 or shape[2] != 3
            or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in shape)
        ):
            raise ValueError("Streaming RGB frames require a positive (H, W, 3) shape")
        return tuple(shape)

    def _run_camera_tasks(self, operation, arguments=None) -> None:
        """Drain every submitted task before propagating an encoding error."""
        error = None
        futures = []
        try:
            for name, stream in self._streams.items():
                args = (stream,) if arguments is None else (stream, arguments[name])
                if self._executor is None:
                    operation(*args)
                else:
                    futures.append(self._executor.submit(operation, *args))
        except BaseException as exc:
            error = exc
        for future in futures:
            try:
                future.result()
            except BaseException as exc:
                if error is None:
                    error = exc
        if error is not None:
            self._fail(f"{type(error).__name__}: {error}")
            raise RuntimeError(f"Recording episode is incomplete: {self._poison}") from error

    def _open_camera(self, camera: _CameraStream, shape) -> None:
        shape = self._validate_shape(shape)
        if camera.container is not None or camera.external is not None:
            if shape != camera.shape:
                raise ValueError(f"Camera {camera.name} changed shape from {camera.shape} to {shape}")
            return
        camera.shape = shape
        if self._backend.name == "h264_rkmpp":
            camera.external = FfmpegCameraEncoder(camera.path, shape, self.fps, self._backend)
            return
        camera.container = self._av.open(str(camera.path), mode="w")
        camera.stream = camera.container.add_stream(
            self._rgb_encoder.vcodec, self.fps,
            options=self._rgb_encoder.get_codec_options(self._encoder_threads, as_strings=True),
        )
        camera.stream.width = shape[1]
        camera.stream.height = shape[0]
        camera.stream.pix_fmt = self._rgb_encoder.pix_fmt
        camera.stream.time_base = self._time_base
        camera.stream.codec_context.time_base = self._time_base
        # Prepare the real codec before writer-ready, without generating a
        # dummy frame or putting artificial samples into the dataset.
        camera.stream.codec_context.open()

    def prepare_episode(self, video_keys, temp_dir, frame_shapes: dict, *, expected_video_info=None) -> None:
        """Open this episode's real empty streams before accepting camera frames."""
        keys = tuple(video_keys)
        if set(frame_shapes) != set(keys):
            raise ValueError("Streaming preparation requires a shape for every video key")
        shapes = {key: self._validate_shape(frame_shapes[key]) for key in keys}
        expected = expected_video_info or {}
        if expected and set(expected) != set(keys):
            raise ValueError("Append validation requires existing video info for every camera")
        self._expected_video_info = {key: dict(info) for key, info in expected.items()}
        self.start_episode(keys, temp_dir)
        self._run_camera_tasks(self._open_camera, shapes)
        logger.info("Streaming video prepared: cameras=%s encoder=%s fps=%d", list(keys), self._backend.name, self.fps)
        if self._backend.name == "h264_rkmpp":
            logger.info("Hardware encoder processes launched; hardware initializes on the first real frame")

    def _encode_camera(self, camera: _CameraStream, image) -> None:
        started = time.monotonic()
        self._open_camera(camera, image.shape)
        if camera.external is not None:
            # The bounded pipe copies the bytes into FFmpeg-owned memory.
            # It cannot retain a reference to the reusable shared RGB slot.
            camera.external.write(image)
        else:
            # PyAV owns the resulting frame; never borrow the shared buffer.
            frame = self._av.VideoFrame.from_ndarray(image, format="rgb24")
            frame.pts = camera.frames
            frame.time_base = self._time_base
            for packet in camera.stream.encode(frame):
                camera.container.mux(packet)
        stats_started = time.monotonic()
        camera.last_encode_s = stats_started - started
        # Match LeRobot's spatial sampling (largest side near 150 pixels),
        # with fixed 256-bin RGB histograms instead of adaptive Python loops.
        largest_side = max(image.shape[:2])
        factor = largest_side // 150 if largest_side >= 300 else 1
        vectors = image[::factor, ::factor, :].reshape(-1, 3)
        camera.stats.update(vectors)
        camera.last_stats_s = time.monotonic() - stats_started
        camera.frames += 1

    def feed_frame(self, video_key: str, image) -> None:
        self._check_open()
        if not self._active:
            raise RuntimeError("Start a streaming episode before feeding frames")
        if self._sealed:
            raise RuntimeError("Video streams have already been finalized")
        try:
            if video_key not in self._streams:
                raise ValueError(f"Unknown streaming camera: {video_key}")
            if video_key in self._pending:
                raise ValueError(f"Camera {video_key} was supplied twice in one frame group")
            if not isinstance(image, self._np.ndarray) or image.dtype != self._np.uint8:
                raise ValueError(f"Camera {video_key} requires a uint8 RGB ndarray")
            shape = self._validate_shape(image.shape)
            expected = self._streams[video_key].shape
            if expected is not None and shape != expected:
                raise ValueError(f"Camera {video_key} changed shape from {expected} to {shape}")
            self._pending[video_key] = image
            if len(self._pending) == len(self._streams):
                # Earlier feed calls only borrow this group's shared views.
                # Dataset.add_frame returns only after the last camera's feed
                # drains every encoder task, so its subsequent ACK is safe.
                try:
                    started = time.monotonic()
                    self._run_camera_tasks(self._encode_camera, self._pending)
                    self.last_group_s = time.monotonic() - started
                    self._groups += 1
                    if self._groups <= 3:
                        logger.info(
                            "Video group %d consumed: total=%.3fs encoder=%s per_camera=%s",
                            self._groups, self.last_group_s, self._backend.name,
                            {name: {"encode_or_pipe_s": round(camera.last_encode_s, 4),
                                    "stats_s": round(camera.last_stats_s, 4)}
                             for name, camera in self._streams.items()},
                        )
                finally:
                    self._pending.clear()
        except BaseException as exc:
            self._fail(f"{type(exc).__name__}: {exc}")
            self._pending.clear()
            raise

    def validate_complete(self, expected_frames: int | None = None) -> None:
        """Let the sink reject a broken episode before LeRobot writes parquet."""
        self._check_open()
        if self._pending:
            self._fail(f"Incomplete camera group: received {list(self._pending)}, expected {list(self._keys)}")
        elif any(camera.frames != self._groups for camera in self._streams.values()):
            self._fail(f"Camera frame counts disagree: {self.frame_counts}")
        elif expected_frames is not None and self._groups != expected_frames:
            self._fail(f"Encoded {self._groups} groups but dataset accepted {expected_frames} frames")
        self._check_open()

    @staticmethod
    def _close_camera(camera: _CameraStream, *, flush=False) -> None:
        if camera.external is not None:
            # Keep the handle on failure so cancel_episode can reap the child.
            if flush:
                camera.external.finish()
            else:
                camera.external.cancel()
            camera.external = None
        if camera.container is None:
            return
        try:
            if flush and camera.stream is not None:
                for packet in camera.stream.encode():
                    camera.container.mux(packet)
        finally:
            try:
                camera.container.close()
            finally:
                camera.container = camera.stream = None

    def _flush_camera(self, camera: _CameraStream) -> None:
        self._close_camera(camera, flush=True)

    def _validate_camera_output(self, camera: _CameraStream) -> None:
        """Probe the finished MP4 without decoding, before dataset commit."""
        with self._av.open(str(camera.path), mode="r") as container:
            stream = container.streams.video[0]
            actual = {
                "video.codec": stream.codec.canonical_name,
                "video.pix_fmt": stream.pix_fmt,
                "video.height": stream.height,
                "video.width": stream.width,
                "video.fps": stream.base_rate,
            }
            if stream.frames != self._groups:
                raise ValueError(
                    f"Video {camera.name} has {stream.frames} frames; expected {self._groups}"
                )
        expected = {
            "video.codec": "h264", "video.height": camera.shape[0],
            "video.width": camera.shape[1], "video.fps": self.fps,
        }
        stored = self._expected_video_info.get(camera.name)
        if stored is not None:
            # Require the actual new output to match the recorded format;
            # yuv420p and yuvj420p are not treated as interchangeable here.
            expected["video.pix_fmt"] = stored["video.pix_fmt"]
            for key in expected:
                if key in stored and stored[key] != expected[key]:
                    raise ValueError(f"Stored {camera.name} {key} disagrees with the session: {stored[key]!r}")
        elif self._backend.name == "libx264":
            expected["video.pix_fmt"] = self._rgb_encoder.pix_fmt
        elif actual["video.pix_fmt"] not in ("yuv420p", "yuvj420p"):
            raise ValueError(f"Unsupported hardware video pixel format: {actual['video.pix_fmt']!r}")
        differences = [
            f"{key}: expected={value!r}, generated={actual[key]!r}"
            for key, value in expected.items() if actual[key] != value
        ]
        if differences:
            raise ValueError(
                f"Video output differs for {camera.name}: " + "; ".join(differences)
                + "; no new episode has been committed; use a new dataset root"
            )

    def _shutdown_executor(self) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=True)
            self._executor = None

    def seal_episode(self, expected_frames: int | None = None) -> None:
        """Flush and check videos before committing the matching dataset rows."""
        self.validate_complete(expected_frames)
        if not self._active:
            raise RuntimeError("No streaming episode is active")
        if not self._sealed:
            self._run_camera_tasks(self._flush_camera)
            self._run_camera_tasks(self._validate_camera_output)
            self._sealed = True

    def finish_episode(self) -> dict[str, tuple[Path, dict | None]]:
        self.validate_complete()
        if not self._active:
            raise RuntimeError("No streaming episode is active")
        if self._groups == 0:
            self.cancel_episode()
            return {}
        try:
            self.seal_episode()
            results = {
                name: (
                    camera.path,
                    camera.stats.get_statistics(),
                )
                for name, camera in self._streams.items()
            }
            self._shutdown_executor()
        except BaseException as exc:
            self._fail(f"Video finalization failed: {type(exc).__name__}: {exc}")
            raise RuntimeError(f"Recording episode is incomplete: {self._poison}") from exc
        # LeRobot now owns these temp videos and removes each parent directory
        # after moving/remuxing it into the standard dataset video layout.
        self._streams.clear()
        self._owned_directories.clear()
        self._active = False
        self._keys, self._root = (), None
        return results

    def cancel_episode(self) -> None:
        """Discard only temporary directories created by this encoder."""
        self._pending.clear()
        errors = []
        try:
            self._shutdown_executor()
        except Exception as exc:
            errors.append(f"Could not stop encoder tasks: {exc}")
        for camera in self._streams.values():
            try:
                self._close_camera(camera)
            except Exception as exc:
                errors.append(f"{camera.name}: {exc}")
        self._streams.clear()
        for directory in tuple(self._owned_directories):
            try:
                shutil.rmtree(directory)
                self._owned_directories.remove(directory)
            except FileNotFoundError:
                self._owned_directories.remove(directory)
            except OSError as exc:
                errors.append(f"Could not remove temporary video directory {directory}: {exc}")
        self._active = False
        self._keys, self._root = (), None
        self._groups = 0
        self._poison = None
        if errors:
            raise RuntimeError("Streaming recording cleanup failed: " + "; ".join(errors))

    def close(self) -> None:
        if self._closed:
            return
        self.cancel_episode()
        self._closed = True


def make_streaming_dataset_class(Dataset, backend=None):
    """Override the fixed LeRobot 0.6.1 encoder factory without patching it."""

    class StreamingRecordingDataset(Dataset):
        @staticmethod
        def _build_streaming_encoder(
            fps, rgb_encoder, depth_encoder, encoder_queue_maxsize, encoder_threads,
        ):
            return SynchronousVideoEncoder(
                fps=fps, rgb_encoder=rgb_encoder, depth_encoder=depth_encoder,
                encoder_threads=encoder_threads,
                backend=backend,
            )

    return StreamingRecordingDataset
