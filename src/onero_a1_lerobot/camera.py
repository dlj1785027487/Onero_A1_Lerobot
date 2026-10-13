"""Background USB MJPEG capture with immutable RGB snapshots and a mock backend.

Only the capture worker owns the V4L2 descriptor and mappings. Stopping normally
takes at most one short select/decode cycle. If a broken driver blocks in an
ioctl, disconnect reports a bounded-time failure instead of blocking the caller
or closing a descriptor concurrently with a pending operation.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .camera_config import UsbCameraConfig

logger = logging.getLogger(__name__)

_READ_WAIT_S = 0.1
_RETRY_WAIT_S = 0.5
_STOP_WAIT_S = 2.0
_CV2_IMPORT_LOCK = threading.Lock()


class CameraError(RuntimeError):
    """A configured camera cannot currently supply a valid, fresh frame."""


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    timestamp: float
    sequence: int
    jpeg: bytes | None = None


class UsbCamera:
    def __init__(self, config: UsbCameraConfig):
        config.validate()
        self.config = config
        self._condition = threading.Condition()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._frame: CameraFrame | None = None
        self._sequence = 0
        self._error: str | None = None
        self._cv2 = None

    @property
    def is_connected(self) -> bool:
        """Lifecycle state; snapshot() reports a temporary camera outage."""
        with self._condition:
            return self._connected

    def preflight(self) -> None:
        """Check configuration and software only; never open a device."""
        self.config.validate()
        if self.config.backend == "mock":
            return
        if not sys.platform.startswith("linux"):
            raise CameraError("USB camera capture requires Linux V4L2")
        if self._cv2 is not None:
            return
        # OpenCV temporarily inserts its own directory into sys.path while
        # bootstrapping. The session calls this before launching subprocesses,
        # so spawn cannot capture that temporary path (which shadows typing).
        with _CV2_IMPORT_LOCK:
            previous_path = sys.path.copy()
            try:
                import cv2

                self._cv2 = cv2
            except Exception as exc:
                raise CameraError(
                    "USB MJPEG decoding requires OpenCV; install this project's camera extra"
                ) from exc
            finally:
                # OpenCV's own restoration is skipped when bootstrap raises.
                # Keep subprocess imports clean on both success and failure.
                sys.path[:] = previous_path

    def connect(self, *, cancelled: Callable[[], bool] | None = None) -> None:
        """Wait for a fresh frame, checking optional startup cancellation."""
        if cancelled is not None and cancelled():
            with self._lifecycle_lock:
                self._disconnect_locked()
            raise CameraError("Camera connection cancelled")
        self.preflight()
        with self._lifecycle_lock:
            if cancelled is not None and cancelled():
                self._disconnect_locked()
                raise CameraError("Camera connection cancelled")
            if self.is_connected:
                return
            if self._thread is not None and self._thread.is_alive():
                raise CameraError("Previous capture worker is still stopping; cannot reconnect")
            self._stop.clear()
            with self._condition:
                self._frame = None
                self._error = None
                self._connected = True
            self._thread = threading.Thread(
                target=self._run,
                name=f"a1-camera-{self.config.device or 'mock'}",
                daemon=True,
            )
            try:
                self._thread.start()
            except BaseException:
                with self._condition:
                    self._connected = False
                self._thread = None
                raise
            try:
                deadline = time.monotonic() + self.config.connect_timeout_s
                with self._condition:
                    while self._frame is None:
                        if cancelled is not None and cancelled():
                            raise CameraError("Camera connection cancelled")
                        remaining = deadline - time.monotonic()
                        if remaining <= 0 or not self._thread.is_alive():
                            break
                        self._condition.wait(min(remaining, 0.1))
                    frame = self._frame
                    error = self._error
                if cancelled is not None and cancelled():
                    raise CameraError("Camera connection cancelled")
                if frame is not None and time.monotonic() - frame.timestamp <= self.config.stale_frame_s:
                    return
                detail = error or "no fresh RGB frame received"
                raise CameraError(
                    f"Camera {self.config.device or 'mock'} did not connect within "
                    f"{self.config.connect_timeout_s:g}s: {detail}"
                )
            except BaseException:
                # An interrupted startup must not leave the USB worker running.
                try:
                    self._disconnect_locked()
                except CameraError:
                    logger.exception("Camera startup cleanup did not finish")
                raise

    def snapshot(self) -> CameraFrame:
        """Return the latest cached frame without waiting for USB I/O."""
        with self._condition:
            if not self._connected:
                raise CameraError("Camera is not connected")
            frame, error = self._frame, self._error
        if frame is None:
            raise CameraError(f"Camera {self.config.device or 'mock'} unavailable: {error or 'no frame'}")
        age = time.monotonic() - frame.timestamp
        if age < 0 or age > self.config.stale_frame_s:
            raise CameraError(
                f"Camera {self.config.device or 'mock'} frame is stale "
                f"(age={age:.3f}s, limit={self.config.stale_frame_s:g}s)"
            )
        return frame

    def disconnect(self) -> None:
        with self._lifecycle_lock:
            self._disconnect_locked()

    def _disconnect_locked(self) -> None:
        self._stop.set()
        with self._condition:
            self._connected = False
            self._frame = None
            self._condition.notify_all()
        thread = self._thread
        if thread is not None:
            thread.join(_STOP_WAIT_S)
            if thread.is_alive():
                raise CameraError(
                    f"Camera worker did not stop within {_STOP_WAIT_S:g}s; "
                    "device cleanup is pending in the worker"
                )
        self._thread = None

    def _publish(self, image: np.ndarray, timestamp: float, jpeg: bytes | None = None) -> None:
        if image.shape != self.config.shape or image.dtype != np.uint8:
            raise CameraError(
                f"Decoded camera frame is {image.shape}/{image.dtype}, "
                f"expected {self.config.shape}/uint8"
            )
        age = time.monotonic() - timestamp
        if age < 0 or age > self.config.stale_frame_s:
            raise CameraError(f"Camera decoding produced an expired frame (age={age:.3f}s)")
        image.setflags(write=False)
        with self._condition:
            if self._stop.is_set():
                return
            self._sequence += 1
            self._frame = CameraFrame(image, timestamp, self._sequence, jpeg)
            self._error = None
            self._condition.notify_all()

    def _invalidate(self, error: Exception) -> None:
        with self._condition:
            self._frame = None
            self._error = f"{type(error).__name__}: {error}"
            self._condition.notify_all()

    def _run(self) -> None:
        try:
            if self.config.backend == "mock":
                self._run_mock()
            else:
                self._run_usb()
        except Exception as exc:
            self._invalidate(exc)
            logger.exception("Camera worker failed: %s", self.config.device or "mock")
        finally:
            with self._condition:
                # The worker no longer serves frames once it returns, for any
                # reason. Clearing _connected here lets connect() start a new
                # worker instead of reporting a connected camera with no thread.
                self._connected = False
                self._frame = None
                self._condition.notify_all()

    def _run_mock(self) -> None:
        x = np.arange(self.config.width, dtype=np.uint32).astype(np.uint8)
        y = np.arange(self.config.height, dtype=np.uint32).astype(np.uint8)[:, None]
        period = 1.0 / self.config.fps
        phase = 0
        while not self._stop.is_set():
            started = time.monotonic()
            image = np.empty(self.config.shape, dtype=np.uint8)
            image[:, :, 0] = x + np.uint8(phase % 256)
            image[:, :, 1] = y + np.uint8((phase * 3) % 256)
            image[:, :, 2] = np.uint8((phase * 7) % 256)
            self._publish(image, time.monotonic())
            phase += 1
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _make_capture(self):
        from .v4l2_camera import V4L2MjpegCapture

        return V4L2MjpegCapture(self.config)

    def _run_usb(self) -> None:
        previous_error = None
        last_warning = float("-inf")
        while not self._stop.is_set():
            capture = None
            try:
                capture = self._make_capture()
                capture.open()
                last_valid = time.monotonic()
                report_started, frames = last_valid, 0
                while not self._stop.is_set():
                    result = capture.read(timeout=_READ_WAIT_S)
                    if result is None:
                        if time.monotonic() - last_valid > self.config.stale_frame_s:
                            raise CameraError("Timed out waiting for a fresh MJPEG frame")
                        continue
                    jpeg, timestamp = result
                    bgr = self._cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8),
                        self._cv2.IMREAD_COLOR | self._cv2.IMREAD_IGNORE_ORIENTATION,
                    )
                    if bgr is None:
                        raise CameraError("Invalid MJPEG image")
                    rgb = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)
                    self._publish(rgb, timestamp, jpeg)
                    last_valid = timestamp
                    frames += 1
                    elapsed = time.monotonic() - report_started
                    if elapsed >= 5.0:
                        logger.info("Camera %s decoded RGB FPS: %.2f", self.config.device, frames / elapsed)
                        report_started, frames = time.monotonic(), 0
            except Exception as exc:
                self._invalidate(exc)
                detail = f"{type(exc).__name__}: {exc}"
                now = time.monotonic()
                if not self._stop.is_set() and (detail != previous_error or now - last_warning >= 5):
                    logger.warning("Camera %s: %s; retrying", self.config.device, detail)
                    previous_error, last_warning = detail, now
            finally:
                if capture is not None:
                    try:
                        capture.close()
                    except Exception as exc:
                        self._invalidate(exc)
                        logger.warning("Camera %s cleanup failed: %s", self.config.device, exc)
            self._stop.wait(_RETRY_WAIT_S)
