"""Stage camera bytes and scalar samples on disk; generate videos after stop."""

from __future__ import annotations

import argparse
import fcntl
import json
import logging
import os
import shutil
import tempfile
import time
from dataclasses import asdict
from pathlib import Path

from .recording_image import RecordingImage, decode_recording_image, pack_recording_image

logger = logging.getLogger(__name__)
_FORMAT = "onero-a1-capture-v1"
_TORCH_CONFIGURED = False


def _write_manifest(directory: Path, manifest: dict) -> None:
    temporary = directory / "session.json.tmp"
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, directory / "session.json")


def _pack_features(features: dict) -> dict:
    result = {}
    for name, feature in features.items():
        if isinstance(feature, tuple):
            result[name] = list(feature)
        elif feature is float:
            result[name] = "float"
        else:
            raise ValueError(f"Unsupported recording feature {name}: {feature}")
    return result


def _unpack_features(features: dict) -> dict:
    return {name: tuple(value) if isinstance(value, list) else float for name, value in features.items()}


def _generate_dataset(
    directory: Path, manifest: dict, *, root=None, repo_id=None, video_encoder=None, progress=None,
) -> int:
    """Only this post-capture path imports Torch/LeRobot or opens encoders."""
    global _TORCH_CONFIGURED
    total = manifest.get("captured_frames")
    if progress is not None:
        progress("loading conversion dependencies", 0, total)
    import torch

    from .episode_recording import LeRobotEpisodeSink, RecordingConfig

    if not _TORCH_CONFIGURED:
        torch.set_num_threads(1)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            logger.warning("Recording PyTorch interop pool was already initialized")
        _TORCH_CONFIGURED = True
    config = RecordingConfig(**manifest["recording"])
    if root is not None:
        config.root = str(Path(root).expanduser().resolve())
    if repo_id is not None:
        config.repo_id = repo_id
    if video_encoder is not None:
        config.video_encoder = video_encoder
    config.validate(starting=True)
    observations = _unpack_features(manifest["observation_features"])
    actions = _unpack_features(manifest["action_features"])
    if progress is not None:
        progress("opening dataset and video encoders", 0, total)
    sink = LeRobotEpisodeSink(config, observations, actions)
    frames = 0
    report_started = time.monotonic()
    try:
        with (directory / "samples.jsonl").open(encoding="utf-8") as journal:
            for line in journal:
                # A terminated disk write may leave an uncommitted final line.
                if not line.endswith("\n"):
                    logger.warning("Ignoring incomplete final sample in %s", directory)
                    break
                sample = json.loads(line)
                if sample["index"] != frames:
                    raise ValueError("Captured sample indices are not consecutive")
                observation = dict(sample["observation"])
                for name, spec in sample["images"].items():
                    relative = Path(spec["file"])
                    if relative.is_absolute() or ".." in relative.parts:
                        raise ValueError("Invalid captured image path")
                    image_path = (directory / relative).resolve()
                    if not image_path.is_relative_to(directory.resolve()):
                        raise ValueError("Captured image points outside its episode")
                    data = image_path.read_bytes()
                    if len(data) != spec["bytes"]:
                        raise ValueError(f"Incomplete captured image: {image_path}")
                    image = RecordingImage(data, tuple(spec["shape"]), spec["encoding"])
                    observation[name] = decode_recording_image(image)
                sink.add(observation, sample["action"], sample["timestamp"])
                frames += 1
                if progress is not None:
                    progress("converting frames", frames, total)
                if time.monotonic() - report_started >= 5:
                    logger.info("Post-record conversion: frames=%d source=%s", frames, directory)
                    report_started = time.monotonic()
        expected_frames = manifest.get("captured_frames")
        if not frames:
            raise ValueError("The capture contains no complete samples to convert")
        if expected_frames is not None and frames != expected_frames:
            raise ValueError(f"Captured journal has {frames} samples, expected {expected_frames}")
        sink.finish(progress=progress)
    except BaseException:
        # Do not save an apparently complete episode from only a prefix if
        # conversion fails. Preserve the entire source for a fresh-root retry.
        try:
            sink.dataset.finalize()
        except Exception:
            logger.exception("Could not close failed dataset conversion")
        raise
    manifest.update(status="saved", converted_frames=frames, dataset_root=str(sink.dataset.root))
    _write_manifest(directory, manifest)
    if progress is not None:
        progress("dataset saved", frames, frames)
    logger.info("Post-record conversion complete: frames=%d dataset=%s", frames, sink.dataset.root)
    return frames


class DeferredEpisodeSink:
    """Fast capture journal in a private directory next to the dataset root."""

    def __init__(self, config, observation_features: dict, action_features: dict):
        root = (Path(config.root).expanduser() if config.root else
                Path.home() / "lerobot_datasets" / config.repo_id).resolve()
        staging = root.parent / ".a1-recordings"
        staging.mkdir(parents=True, exist_ok=True)
        self.frames = 0
        self._finished = False
        self._journal = None
        self._lock = None
        self._shapes = {key: shape for key, shape in observation_features.items() if isinstance(shape, tuple)}
        self._manifest = {
            "format": _FORMAT, "status": "capturing", "recording": {**asdict(config), "root": str(root)},
            "observation_features": _pack_features(observation_features),
            "action_features": _pack_features(action_features),
        }
        self.directory = Path(tempfile.mkdtemp(prefix=f"{root.name}-", dir=staging))
        try:
            self._lock = (self.directory / "capture.lock").open("a+b")
            fcntl.flock(self._lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            (self.directory / "frames").mkdir()
            _write_manifest(self.directory, self._manifest)
            self._journal = (self.directory / "samples.jsonl").open("x", encoding="utf-8")
        except BaseException:
            if self._lock is not None:
                self._lock.close()
            shutil.rmtree(self.directory)
            raise
        logger.info("Recording capture ready: raw frames + sample journal at %s; video generation after stop", self.directory)

    def add(self, observation: dict, action: dict, timestamp: float) -> None:
        if self._finished:
            raise RuntimeError("Capture has already stopped")
        images = {}
        for index, (name, shape) in enumerate(self._shapes.items()):
            image = pack_recording_image(observation[name], shape)
            extension = "jpg" if image.encoding == "jpeg" else "rgb"
            relative = f"frames/{self.frames:08d}-{index:03d}.{extension}"
            # This private path is reused only for an uncommitted failed group.
            with (self.directory / relative).open("wb") as stream:
                stream.write(image.data)
            images[name] = {"file": relative, "shape": list(shape), "encoding": image.encoding, "bytes": image.nbytes}
        sample = {
            "index": self.frames, "timestamp": float(timestamp), "images": images,
            "observation": {name: float(value) for name, value in observation.items() if name not in self._shapes},
            "action": {name: float(value) for name, value in action.items()},
        }
        self._journal.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
        self._journal.flush()
        self.frames += 1

    def finish(self, *, progress=None) -> None:
        if self._finished:
            return
        self._finished = True
        try:
            if progress is not None:
                progress("closing capture journal", self.frames, self.frames)
            self._journal.close()
            if self.frames:
                self._manifest.update(status="generating", captured_frames=self.frames)
                _write_manifest(self.directory, self._manifest)
                logger.info("Recording capture finished: frames=%d; generating videos from %s", self.frames, self.directory)
                _generate_dataset(self.directory, self._manifest, progress=progress)
            try:
                if progress is not None:
                    progress("removing saved capture files", self.frames, self.frames)
                shutil.rmtree(self.directory)
            except OSError:
                logger.warning("Dataset saved; could not remove capture source %s", self.directory, exc_info=True)
        except BaseException:
            logger.error("Capture source retained at %s; retry conversion with a new dataset root", self.directory)
            raise
        finally:
            self._lock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a LeRobot dataset from a retained A1 capture; no hardware is opened")
    parser.add_argument("capture", type=Path, help="Directory printed as 'Capture source retained'")
    parser.add_argument("--root", required=True, help="Fresh output dataset directory for this recovery attempt")
    parser.add_argument("--repo-id", default=None)
    parser.add_argument("--video-encoder", choices=("auto", "libx264", "h264_rkmpp"), default=None)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    directory = args.capture.expanduser().resolve()
    if Path(args.root).expanduser().exists():
        raise SystemExit("Recovery requires a new --root to avoid appending a duplicate or incomplete episode")
    with (directory / "capture.lock").open("a+b") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("This capture is still recording or being converted") from None
        manifest = json.loads((directory / "session.json").read_text(encoding="utf-8"))
        if manifest.get("format") != _FORMAT:
            raise SystemExit("Not an A1 capture directory")
        if manifest.get("status") == "saved":
            raise SystemExit("This capture has already been converted successfully")
        _generate_dataset(
            directory, manifest, root=args.root, repo_id=args.repo_id, video_encoder=args.video_encoder,
        )
        # Manual recovery keeps source files for the operator to inspect.


if __name__ == "__main__":
    main()
