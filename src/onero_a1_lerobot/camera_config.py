"""Serializable USB camera settings; no capture or decoder dependencies."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import PurePosixPath


@dataclass
class UsbCameraConfig:
    backend: str = "usb"
    device: str | None = None
    width: int = 640
    height: int = 480
    fps: int = 30
    auto_exposure: bool | None = None
    exposure_absolute: int | None = None
    connect_timeout_s: float = 5.0
    stale_frame_s: float = 0.5

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.backend not in ("usb", "mock"):
            raise ValueError("Camera backend must be usb or mock")
        if self.device is not None and (
            not isinstance(self.device, str) or not self.device or "\0" in self.device
        ):
            raise ValueError("Camera device must be a nonempty device path or None")
        if self.backend == "usb":
            if not self.device:
                raise ValueError("USB camera requires an explicit /dev/... device path")
            path = PurePosixPath(self.device)
            if len(path.parts) < 3 or path.parts[:2] != ("/", "dev") or ".." in path.parts:
                raise ValueError("USB camera device must be a /dev/... path")
        for name in ("width", "height", "fps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= 0xFFFFFFFF:
                raise ValueError(f"Camera {name} must be a positive 32-bit integer")
        if self.auto_exposure is not None and not isinstance(self.auto_exposure, bool):
            raise ValueError("Camera auto_exposure must be a boolean or None")
        if self.backend == "mock" and self.auto_exposure is not None:
            raise ValueError("Mock cameras must not specify V4L2 exposure controls")
        if self.exposure_absolute is not None:
            if (
                isinstance(self.exposure_absolute, bool)
                or not isinstance(self.exposure_absolute, int)
                or self.exposure_absolute <= 0
            ):
                raise ValueError("Camera exposure_absolute must be a positive integer of 100 us units")
            if self.auto_exposure is not False:
                raise ValueError(
                    "Camera exposure_absolute requires auto_exposure false, "
                    "because auto exposure ignores a fixed exposure time"
                )
            if self.exposure_absolute * self.fps > 10_000:
                raise ValueError(
                    f"Camera exposure_absolute {self.exposure_absolute} "
                    f"({self.exposure_absolute / 10.0:.1f} ms) exceeds one {self.fps} fps frame period, "
                    "so the camera would deliver fewer frames than the dataset assumes"
                )
        for name in ("connect_timeout_s", "stale_frame_s"):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"Camera {name} must be finite and positive")

    @property
    def shape(self) -> tuple[int, int, int]:
        return self.height, self.width, 3
