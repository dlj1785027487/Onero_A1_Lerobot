"""Linux V4L2 MJPEG transport using nonblocking, memory-mapped capture.

Only the owning camera worker should call this transport. No camera is opened
at import time, and JPEG decoding belongs to the caller.
"""

from __future__ import annotations

import ctypes
import errno
import fcntl
import logging
import math
import mmap
import os
import select
import time
from typing import Protocol

logger = logging.getLogger(__name__)

_U8 = ctypes.c_uint8
_U32 = ctypes.c_uint32
_CAP_VIDEO_CAPTURE = 0x00000001
_CAP_STREAMING = 0x04000000
_CAP_DEVICE_CAPS = 0x80000000
_CAP_TIMEPERFRAME = 0x1000
_BUF_TYPE_VIDEO_CAPTURE = 1
_MEMORY_MMAP = 1
_BUF_FLAG_ERROR = 0x00000040
_PIX_FMT_MJPEG = int.from_bytes(b"MJPG", "little")


class _Capability(ctypes.Structure):
    _fields_ = [
        ("driver", _U8 * 16),
        ("card", _U8 * 32),
        ("bus_info", _U8 * 32),
        ("version", _U32),
        ("capabilities", _U32),
        ("device_caps", _U32),
        ("reserved", _U32 * 3),
    ]


class _PixFormat(ctypes.Structure):
    _fields_ = [(name, _U32) for name in (
        "width", "height", "pixelformat", "field", "bytesperline", "sizeimage",
        "colorspace", "priv", "flags", "ycbcr_enc", "quantization", "xfer_func",
    )]


class _FormatUnion(ctypes.Union):
    # v4l2_window contains pointers; raw_data alone loses the union's ABI alignment.
    _fields_ = [
        ("pix", _PixFormat),
        ("raw_data", _U8 * 200),
        ("_pointer_alignment", ctypes.c_void_p),
    ]


class _Format(ctypes.Structure):
    _fields_ = [("type", _U32), ("fmt", _FormatUnion)]


class _Fraction(ctypes.Structure):
    _fields_ = [("numerator", _U32), ("denominator", _U32)]


class _CaptureParm(ctypes.Structure):
    _fields_ = [
        ("capability", _U32),
        ("capturemode", _U32),
        ("timeperframe", _Fraction),
        ("extendedmode", _U32),
        ("readbuffers", _U32),
        ("reserved", _U32 * 4),
    ]


class _StreamParmUnion(ctypes.Union):
    _fields_ = [("capture", _CaptureParm), ("raw_data", _U8 * 200)]


class _StreamParm(ctypes.Structure):
    _fields_ = [("type", _U32), ("parm", _StreamParmUnion)]


class _RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", _U32),
        ("type", _U32),
        ("memory", _U32),
        ("capabilities", _U32),
        ("flags", _U8),
        ("reserved", _U8 * 3),
    ]


class _Control(ctypes.Structure):
    _fields_ = [("id", _U32), ("value", ctypes.c_int32)]


# Camera-class controls from videodev2.h: V4L2_CID_CAMERA_CLASS_BASE is
# V4L2_CTRL_CLASS_CAMERA (0x009A0000) | 0x900, and exposure_absolute is in
# units of 100 us. The enum values are V4L2_EXPOSURE_AUTO / V4L2_EXPOSURE_MANUAL.
_CID_EXPOSURE_AUTO = 0x009A0901
_CID_EXPOSURE_ABSOLUTE = 0x009A0902
_EXPOSURE_AUTO = 0
_EXPOSURE_MANUAL = 1


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [
        ("type", _U32), ("flags", _U32),
        ("frames", _U8), ("seconds", _U8), ("minutes", _U8), ("hours", _U8),
        ("userbits", _U8 * 4),
    ]


class _BufferMemory(ctypes.Union):
    _fields_ = [
        ("offset", _U32),
        ("userptr", ctypes.c_ulong),
        ("planes", ctypes.c_void_p),
        ("fd", ctypes.c_int32),
    ]


class _Buffer(ctypes.Structure):
    _fields_ = [
        ("index", _U32),
        ("type", _U32),
        ("bytesused", _U32),
        ("flags", _U32),
        ("field", _U32),
        ("timestamp", _Timeval),
        ("timecode", _Timecode),
        ("sequence", _U32),
        ("memory", _U32),
        ("m", _BufferMemory),
        ("length", _U32),
        ("reserved2", _U32),
        ("request_fd", ctypes.c_int32),
    ]


def _ioc(direction: int, number: int, structure: type) -> int:
    """Linux asm-generic ioctl encoding (used by x86 and ARM)."""
    return (direction << 30) | (ctypes.sizeof(structure) << 16) | (ord("V") << 8) | number


_QUERYCAP = _ioc(2, 0, _Capability)
_G_FMT = _ioc(3, 4, _Format)
_S_FMT = _ioc(3, 5, _Format)
_REQBUFS = _ioc(3, 8, _RequestBuffers)
_QUERYBUF = _ioc(3, 9, _Buffer)
_QBUF = _ioc(3, 15, _Buffer)
_DQBUF = _ioc(3, 17, _Buffer)
_STREAMON = _ioc(1, 18, ctypes.c_int)
_STREAMOFF = _ioc(1, 19, ctypes.c_int)
_G_PARM = _ioc(3, 21, _StreamParm)
_S_PARM = _ioc(3, 22, _StreamParm)
_S_CTRL = _ioc(3, 28, _Control)


class _CameraConfig(Protocol):
    device: str
    width: int
    height: int
    fps: int
    auto_exposure: bool | None
    exposure_absolute: int | None


class V4L2MjpegCapture:
    """Own one V4L2 stream; all methods must run on the same worker thread."""

    def __init__(self, config: _CameraConfig):
        self.config = config
        self._fd: int | None = None
        self._buffers: list[mmap.mmap] = []
        self._streaming = False

    @property
    def is_open(self) -> bool:
        return self._fd is not None and self._streaming

    def _ioctl(self, request: int, argument) -> None:
        if self._fd is None:
            raise RuntimeError("V4L2 camera is closed")
        fcntl.ioctl(self._fd, request, argument, True)

    @staticmethod
    def _buffer(index: int = 0) -> _Buffer:
        return _Buffer(index=index, type=_BUF_TYPE_VIDEO_CAPTURE, memory=_MEMORY_MMAP)

    def _check_format(self, fmt: _Format) -> None:
        actual = fmt.fmt.pix
        if (actual.width, actual.height, actual.pixelformat) != (
            self.config.width, self.config.height, _PIX_FMT_MJPEG,
        ):
            fourcc = int(actual.pixelformat).to_bytes(4, "little").decode("ascii", "replace")
            raise RuntimeError(
                f"{self.config.device}: requested MJPG {self.config.width}x{self.config.height}, "
                f"driver selected {fourcc} {actual.width}x{actual.height}"
            )

    def _configure_fps(self) -> None:
        fps = self.config.fps
        if isinstance(fps, bool) or not isinstance(fps, int) or not 0 < fps <= 0xFFFFFFFF:
            raise ValueError("Camera fps must be a positive 32-bit integer")
        params = _StreamParm(type=_BUF_TYPE_VIDEO_CAPTURE)
        params.parm.capture.timeperframe = _Fraction(1, fps)
        unsupported = (errno.EINVAL, errno.ENOTTY, errno.EOPNOTSUPP)
        try:
            self._ioctl(_S_PARM, params)
        except OSError as exc:
            if exc.errno not in unsupported:
                raise
            logger.warning("%s: setting capture fps is unsupported (%s)", self.config.device, exc)
        actual = _StreamParm(type=_BUF_TYPE_VIDEO_CAPTURE)
        try:
            self._ioctl(_G_PARM, actual)
        except OSError as exc:
            if exc.errno not in unsupported:
                raise
            logger.warning("%s: reading capture fps is unsupported (%s)", self.config.device, exc)
            return
        capture = actual.parm.capture
        interval = capture.timeperframe
        if interval.numerator and interval.denominator:
            actual_fps = interval.denominator / interval.numerator
            logger.info("%s: requested %.3f fps, driver reports %.3f fps",
                        self.config.device, fps, actual_fps)
            if not math.isclose(fps, actual_fps, rel_tol=0.01):
                logger.warning("%s: capture fps differs from requested rate", self.config.device)
        else:
            logger.warning("%s: driver did not report a valid capture fps", self.config.device)
        if not capture.capability & _CAP_TIMEPERFRAME:
            logger.warning("%s: driver does not advertise adjustable capture fps", self.config.device)

    def _set_control(self, control_id: int, value: int, name: str) -> None:
        control = _Control(id=control_id, value=value)
        try:
            self._ioctl(_S_CTRL, control)
        except OSError as exc:
            if exc.errno not in (errno.EINVAL, errno.ENOTTY, errno.EOPNOTSUPP):
                raise
            logger.warning("%s: setting %s is unsupported (%s)", self.config.device, name, exc)
            return
        logger.info("%s: %s=%d", self.config.device, name, value)

    def _configure_controls(self) -> None:
        """Pin the exposure controls before streaming starts.

        A camera left in auto exposure lengthens its exposure time when the
        scene darkens. Past one frame period that caps the delivered rate below
        ``fps``, and a recorder sampling at ``fps`` then reuses the previous
        frame, so the dataset carries duplicated camera samples.
        """
        auto_exposure = getattr(self.config, "auto_exposure", None)
        exposure_absolute = getattr(self.config, "exposure_absolute", None)
        if auto_exposure is not None:
            self._set_control(_CID_EXPOSURE_AUTO,
                              _EXPOSURE_AUTO if auto_exposure else _EXPOSURE_MANUAL,
                              "auto_exposure")
        if exposure_absolute is not None:
            self._set_control(_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute")

    def open(self) -> None:
        """Configure the requested MJPG dimensions and start four mmap buffers."""
        if self.is_open:
            return
        try:
            self._fd = os.open(self.config.device, os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)
            capability = _Capability()
            self._ioctl(_QUERYCAP, capability)
            caps = (capability.device_caps if capability.capabilities & _CAP_DEVICE_CAPS
                    else capability.capabilities)
            required = _CAP_VIDEO_CAPTURE | _CAP_STREAMING
            if caps & required != required:
                raise RuntimeError(f"{self.config.device}: requires V4L2 capture and streaming support")

            fmt = _Format(type=_BUF_TYPE_VIDEO_CAPTURE)
            fmt.fmt.pix.width = self.config.width
            fmt.fmt.pix.height = self.config.height
            fmt.fmt.pix.pixelformat = _PIX_FMT_MJPEG
            self._ioctl(_S_FMT, fmt)
            self._check_format(fmt)
            actual = _Format(type=_BUF_TYPE_VIDEO_CAPTURE)
            self._ioctl(_G_FMT, actual)
            self._check_format(actual)
            logger.info("%s: negotiated MJPG %dx%d", self.config.device,
                        actual.fmt.pix.width, actual.fmt.pix.height)
            self._configure_fps()
            self._configure_controls()

            request = _RequestBuffers(count=4, type=_BUF_TYPE_VIDEO_CAPTURE, memory=_MEMORY_MMAP)
            self._ioctl(_REQBUFS, request)
            if request.count < 2:
                raise RuntimeError(f"{self.config.device}: V4L2 requires at least two mmap buffers")
            for index in range(request.count):
                buffer = self._buffer(index)
                self._ioctl(_QUERYBUF, buffer)
                if buffer.length == 0:
                    raise RuntimeError(f"{self.config.device}: driver returned an empty mmap buffer")
                mapped = mmap.mmap(self._fd, buffer.length, flags=mmap.MAP_SHARED,
                                   prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=buffer.m.offset)
                self._buffers.append(mapped)
                self._ioctl(_QBUF, self._buffer(index))
            self._ioctl(_STREAMON, ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))
            self._streaming = True
        except BaseException:
            self.close()
            raise

    def read(self, timeout: float = 0.1) -> tuple[bytes, float] | None:
        """Copy the newest ready JPEG and its host monotonic dequeue time.

        At most one buffer pool is drained per call. ``None`` means no frame was
        ready or the wait/dequeue was interrupted. Driver errors are propagated
        so the owning worker can reconnect the camera.
        """
        if not self.is_open:
            raise RuntimeError("V4L2 camera is closed")
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("Camera read timeout must be finite and nonnegative")
        try:
            ready, _, _ = select.select([self._fd], [], [], timeout)
        except OSError as exc:
            if exc.errno == errno.EINTR:
                return None
            raise
        if not ready:
            return None
        latest = None
        for _ in range(len(self._buffers)):
            buffer = self._buffer()
            try:
                self._ioctl(_DQBUF, buffer)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EINTR):
                    break
                raise
            timestamp = time.monotonic()
            try:
                # Inside the try so the dequeued buffer is always re-queued,
                # even when the index is invalid.
                if buffer.index >= len(self._buffers):
                    raise RuntimeError(
                        f"{self.config.device}: invalid V4L2 buffer index {buffer.index}"
                    )
                mapped = self._buffers[buffer.index]
                if buffer.flags & _BUF_FLAG_ERROR:
                    raise RuntimeError(f"{self.config.device}: V4L2 reported a damaged frame")
                if not 0 < buffer.bytesused <= len(mapped):
                    raise RuntimeError(
                        f"{self.config.device}: invalid V4L2 frame size {buffer.bytesused}"
                    )
                latest = (mapped[:buffer.bytesused], timestamp)
            finally:
                self._ioctl(_QBUF, self._buffer(buffer.index))
        return latest

    def close(self) -> None:
        """Release stream, mappings and fd, including partial initialization."""
        if self._fd is not None and self._streaming:
            try:
                self._ioctl(_STREAMOFF, ctypes.c_int(_BUF_TYPE_VIDEO_CAPTURE))
            except OSError:
                logger.debug("%s: stream stop failed during cleanup", self.config.device, exc_info=True)
        self._streaming = False
        buffers, self._buffers = self._buffers, []
        for mapped in buffers:
            try:
                mapped.close()
            except (OSError, BufferError):
                logger.warning("%s: mmap cleanup failed", self.config.device, exc_info=True)
        fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                logger.debug("%s: fd cleanup failed", self.config.device, exc_info=True)
