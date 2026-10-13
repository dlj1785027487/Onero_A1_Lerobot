"""Immutable image payloads for disk staging; no video or ML imports."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RecordingImage:
    data: bytes
    shape: tuple[int, int, int]
    encoding: str = "jpeg"

    @property
    def nbytes(self) -> int:
        return len(self.data)

    def validate(self, expected_shape) -> None:
        if self.shape != tuple(expected_shape) or len(self.shape) != 3 or self.shape[2] != 3:
            raise ValueError(f"Recording image shape {self.shape} differs from {expected_shape}")
        if not isinstance(self.data, bytes) or not self.data:
            raise ValueError("Recording image must contain immutable bytes")
        if self.encoding == "jpeg":
            if not self.data.startswith(b"\xff\xd8"):
                raise ValueError("Invalid camera JPEG header")
        elif self.encoding == "rgb24":
            if len(self.data) != math.prod(self.shape):
                raise ValueError("Raw RGB byte count does not match image shape")
        else:
            raise ValueError(f"Unsupported recording image encoding: {self.encoding}")


def pack_recording_image(image, shape) -> RecordingImage:
    """Called in the handoff thread, never copy full RGB in the control loop."""
    if isinstance(image, RecordingImage):
        image.validate(shape)
        return image
    import numpy as np

    if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != shape:
        raise ValueError(f"Recording image requires uint8 RGB with shape {shape}")
    return RecordingImage(image.tobytes(order="C"), shape, "rgb24")


def decode_recording_image(image: RecordingImage):
    """Decode only when generating the dataset after capture has stopped."""
    import numpy as np

    image.validate(image.shape)
    if image.encoding == "rgb24":
        return np.frombuffer(image.data, dtype=np.uint8).reshape(image.shape)
    from io import BytesIO

    from PIL import Image

    with Image.open(BytesIO(image.data)) as decoded:
        if (decoded.height, decoded.width, 3) != image.shape:
            raise ValueError("Stored JPEG dimensions differ from the camera configuration")
        return np.array(decoded.convert("RGB"), dtype=np.uint8)
