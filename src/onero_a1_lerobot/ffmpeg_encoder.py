"""Optional system-FFmpeg Rockchip encoder, owned by the recording process."""

from __future__ import annotations

import logging
import os
import re
import select
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class VideoBackend:
    name: str
    executable: str | None = None
    pixel_format: str = "rgb24"

    @property
    def signature(self) -> dict:
        if self.name == "h264_rkmpp":
            return {"name": self.name, "rc_mode": "CQP", "qp": 23, "input_format": self.pixel_format}
        return {"name": "libx264"}


def resolve_video_backend(requested: str) -> VideoBackend:
    """Inspect installed encoder metadata; never open a video device here."""
    if requested not in ("auto", "libx264", "h264_rkmpp"):
        raise ValueError("recording.video_encoder must be auto, libx264 or h264_rkmpp")
    if requested == "libx264":
        return VideoBackend("libx264")
    device = Path("/dev/mpp_service")
    executable = shutil.which("ffmpeg")
    available = False
    pixel_format = "rgb24"
    if executable and device.is_char_device():
        try:
            result = subprocess.run(
                [executable, "-hide_banner", "-h", "encoder=h264_rkmpp"],
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=5, check=False,
            )
            formats = re.search(r"Supported pixel formats:\s*([^\r\n]+)", result.stdout)
            advertised = formats.group(1).split() if formats else []
            available = result.returncode == 0 and "Encoder h264_rkmpp" in result.stdout
            if "rgb24" in advertised:
                pixel_format = "rgb24"
            elif "nv12" in advertised:
                pixel_format = "nv12"
            else:
                available = False
        except (OSError, subprocess.TimeoutExpired) as exc:
            if requested == "h264_rkmpp":
                raise RuntimeError(f"Cannot inspect system FFmpeg's h264_rkmpp encoder: {exc}") from exc
    if available:
        if not os.access(device, os.R_OK | os.W_OK):
            if requested == "h264_rkmpp":
                raise PermissionError("h264_rkmpp requires read/write access to /dev/mpp_service")
            # auto must fall back to the software encoder: a permission problem
            # on /dev/mpp_service would otherwise fail the whole post-record
            # conversion even though libx264 is available.
            logger.warning(
                "h264_rkmpp is available but /dev/mpp_service is not readable/writable; "
                "falling back to libx264"
            )
            available = False
        else:
            logger.info("Recording encoder selected: h264_rkmpp (%s), input=%s", executable, pixel_format)
            return VideoBackend("h264_rkmpp", executable, pixel_format)
    if requested == "h264_rkmpp":
        raise RuntimeError("h264_rkmpp requires /dev/mpp_service and a system FFmpeg with Rockchip encoding")
    logger.info("Recording encoder selected: libx264 (no supported Rockchip encoder detected)")
    return VideoBackend("libx264")


class FfmpegCameraEncoder:
    """One bounded rawvideo pipe to a hardware encoder; no frame dropping."""

    def __init__(self, path: Path, shape: tuple[int, int, int], fps: int, backend: VideoBackend):
        if backend.name != "h264_rkmpp" or not backend.executable:
            raise ValueError("A resolved system h264_rkmpp encoder is required")
        self.path, self.shape, self.fps = Path(path), shape, fps
        self.frames = 0
        self._finished = False
        self._log_path = self.path.with_suffix(".ffmpeg.log")
        self._process: subprocess.Popen | None = None
        command = [
            backend.executable, "-hide_banner", "-loglevel", "error", "-nostats", "-nostdin",
            "-y", "-filter_threads", "1", "-threads", "1",
            "-thread_queue_size", "1", "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{shape[1]}x{shape[0]}", "-framerate", str(fps), "-i", "pipe:0",
            "-map", "0:v:0", "-an", "-sn", "-dn", "-c:v", "h264_rkmpp", "-threads", "1",
            "-pix_fmt", backend.pixel_format, "-rc_mode", "CQP", "-qp_init", "23",
            "-qp_min", "23", "-qp_max", "23", "-qp_min_i", "23", "-qp_max_i", "23",
            "-g", str(fps), "-bf", "0", "-fps_mode", "passthrough", str(self.path),
        ]
        try:
            with self._log_path.open("wb") as errors:
                self._process = subprocess.Popen(
                    command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=errors,
                    bufsize=0, start_new_session=True,
                )
            fd = self._process.stdin.fileno()
            os.set_blocking(fd, False)
            # A larger pipe reduces syscall overhead, but remains bounded and
            # optional: kernels may reject enlarging it for unprivileged users.
            try:
                import fcntl

                fcntl.fcntl(fd, fcntl.F_SETPIPE_SZ, 256 * 1024)
            except (AttributeError, OSError):
                pass
            logger.info("Hardware video encoder PID=%d launched for %s", self._process.pid, self.path.name)
        except BaseException:
            self.cancel()
            raise

    def _failure(self, message: str) -> RuntimeError:
        try:
            with self._log_path.open("rb") as stream:
                stream.seek(max(0, os.fstat(stream.fileno()).st_size - 4096))
                detail = stream.read().decode("utf-8", errors="replace").strip()
        except OSError:
            detail = "no FFmpeg error output"
        return RuntimeError(f"{message}: {detail}")

    def check_running(self) -> None:
        if self._process is None or self._finished:
            raise RuntimeError("Hardware video encoder is not accepting frames")
        status = self._process.poll()
        if status is not None:
            raise self._failure(f"h264_rkmpp exited unexpectedly (exitcode={status})")

    def write(self, image) -> None:
        self.check_running()
        if tuple(image.shape) != self.shape or not image.flags.c_contiguous:
            raise ValueError("Hardware encoder requires a contiguous RGB frame with the configured shape")
        data = memoryview(image).cast("B")
        fd = self._process.stdin.fileno()
        poller = select.poll()
        poller.register(fd, select.POLLOUT | select.POLLERR | select.POLLHUP)
        deadline = time.monotonic() + 30.0
        while data:
            self.check_running()
            if time.monotonic() >= deadline:
                raise self._failure("Hardware video input stalled for 30 seconds")
            events = poller.poll(100)
            if not events:
                continue
            if events[0][1] & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                raise self._failure("Hardware video input pipe closed")
            try:
                written = os.write(fd, data)
            except BlockingIOError:
                continue
            except OSError as exc:
                raise self._failure(f"Hardware video input failed ({exc})") from exc
            if written <= 0:
                raise self._failure("Hardware video input accepted no bytes")
            data = data[written:]
        # FFmpeg owns all bytes at this point; it cannot refer back to SHM.
        self.frames += 1

    def finish(self) -> None:
        if self._finished:
            return
        self.check_running()
        self._process.stdin.close()
        try:
            status = self._process.wait(timeout=30.0)
        except subprocess.TimeoutExpired as exc:
            self.cancel()
            raise self._failure("Hardware video encoder did not finish within 30 seconds") from exc
        if status != 0:
            raise self._failure(f"Hardware video encoder failed (exitcode={status})")
        # Check the real MP4 sample count, without decoding or generating frames.
        import av

        with av.open(str(self.path), mode="r") as container:
            stream = container.streams.video[0]
            if stream.frames != self.frames or (stream.height, stream.width, 3) != self.shape:
                raise self._failure(
                    f"Hardware video output mismatch: expected {self.frames} frames/{self.shape}, "
                    f"got {stream.frames} frames/{(stream.height, stream.width, 3)}"
                )
        self._finished = True

    def cancel(self) -> None:
        process = self._process
        if process is None:
            return
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        self._finished = True
