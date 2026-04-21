"""Video streaming types and data structures."""

from dataclasses import dataclass, field
from enum import IntEnum
import time
from typing import Callable, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from pyhulax.config import get_config


class StreamState(IntEnum):
    """Video stream connection state."""
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    STREAMING = 3
    ERROR = 4
    STOPPED = 5


@dataclass
class StreamConfig:
    """Video stream configuration."""
    drone_ip: str = field(default_factory=lambda: get_config().network.drone_ip)
    drone_id: int = 1
    timeout: float = field(default_factory=lambda: get_config().video.timeout_sec)
    buffer_size: int = field(default_factory=lambda: get_config().video.buffer_size)

    @property
    def rtp_port(self) -> int:
        """Calculate RTP port from drone ID."""
        return get_config().network.rtp_base_port + (self.drone_id * 2)

    def generate_sdp(self, local_ip: str = "0.0.0.0") -> str:
        """
        Generate SDP content for RTP stream.

        Args:
            local_ip: Local IP address to receive stream on (default 0.0.0.0 for any)

        Returns:
            SDP file content string

        Note:
            Format based on C# Unity app's SDP generation, with required
            SDP header fields for FFmpeg/PyAV compatibility.
        """
        # SDP format matching working taskcontroller implementation
        return (
            f"v=0\n"
            f"o=- 0 0 IN IP4 0.0.0.0\n"
            f"s=DroneStream\n"
            f"c=IN IP4 0.0.0.0\n"
            f"t=0 0\n"
            f"m=video {self.rtp_port} RTP/AVP 98\n"
            f"a=rtpmap:98 H264/90000\n"
        )


@dataclass
class BoundingBox:
    """Bounding box for detected object."""
    x: int  # Top-left X coordinate
    y: int  # Top-left Y coordinate
    width: int
    height: int

    @property
    def x2(self) -> int:
        """Bottom-right X coordinate."""
        return self.x + self.width

    @property
    def y2(self) -> int:
        """Bottom-right Y coordinate."""
        return self.y + self.height

    @property
    def center(self) -> Tuple[int, int]:
        """Center point (x, y)."""
        return self.x + self.width // 2, self.y + self.height // 2

    @property
    def area(self) -> int:
        """Area in pixels."""
        return self.width * self.height

    def to_tuple(self) -> Tuple[int, int, int, int]:
        """Return as (x, y, w, h) tuple."""
        return self.x, self.y, self.width, self.height

    def to_xyxy(self) -> Tuple[int, int, int, int]:
        """Return as (x1, y1, x2, y2) tuple."""
        return self.x, self.y, self.x2, self.y2


@dataclass
class Detection:
    """Single object detection result."""
    label: str
    confidence: float
    bbox: BoundingBox
    class_id: Optional[int] = None
    color: Tuple[int, int, int] = (0, 255, 0)  # BGR color for drawing
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        """Assign default color based on class_id if not specified."""
        if self.class_id is not None and self.color == (0, 255, 0):
            # Generate consistent color from class_id
            hue = (self.class_id * 37) % 180
            self.color = self._hsv_to_bgr(hue, 255, 255)

    @staticmethod
    def _hsv_to_bgr(h: int, s: int, v: int) -> Tuple[int, int, int]:
        """Convert HSV to BGR color."""
        import cv2
        hsv = np.array([[[h, s, v]]], dtype=np.uint8)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        b, g, r = int(bgr[0, 0, 0]), int(bgr[0, 0, 1]), int(bgr[0, 0, 2])
        return (b, g, r)


@dataclass
class Frame:
    """
    Video frame with optional detections and metadata.

    The frame flows through the callback pipeline, allowing each
    callback to add detections or modify the image.
    """
    image: NDArray[np.uint8]  # BGR image (OpenCV format)
    timestamp: float = field(default_factory=time.time)
    frame_number: int = 0
    detections: List[Detection] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)

    @property
    def shape(self) -> Tuple[int, int, int]:
        """Image shape (height, width, channels)."""
        h, w, c = self.image.shape
        return (h, w, c)

    @property
    def height(self) -> int:
        """Image height in pixels."""
        return self.image.shape[0]

    @property
    def width(self) -> int:
        """Image width in pixels."""
        return self.image.shape[1]

    @property
    def size(self) -> Tuple[int, int]:
        """Image size as (width, height)."""
        return self.width, self.height

    def copy(self) -> "Frame":
        """Create a deep copy of the frame."""
        return Frame(
            image=self.image.copy(),
            timestamp=self.timestamp,
            frame_number=self.frame_number,
            detections=list(self.detections),
            metadata=dict(self.metadata),
        )

    def draw_detections(self, thickness: int = 2, font_scale: float = 0.6) -> NDArray[np.uint8]:
        """
        Draw detection boxes and labels on a copy of the image.

        Args:
            thickness: Line thickness for boxes
            font_scale: Font scale for labels

        Returns:
            Annotated image copy
        """
        import cv2

        annotated = self.image.copy()

        for det in self.detections:
            x1, y1, x2, y2 = det.bbox.to_xyxy()
            color = det.color

            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            # Prepare label text
            label = f"{det.label}: {det.confidence:.2f}"

            # Get text size for background
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )

            # Draw label background
            cv2.rectangle(
                annotated,
                (x1, y1 - text_h - baseline - 4),
                (x1 + text_w, y1),
                color,
                -1  # Filled
            )

            # Draw label text
            cv2.putText(
                annotated,
                label,
                (x1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),  # White text
                thickness,
            )

        return annotated

    def to_rgb(self) -> "NDArray[np.uint8]":
        """Convert BGR image to RGB."""
        import cv2
        return cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)  # type: ignore[return-value]

    def to_jpeg(self, quality: int = 85) -> bytes:
        """
        Encode frame as JPEG bytes.

        Args:
            quality: JPEG quality (0-100)

        Returns:
            JPEG encoded bytes
        """
        import cv2
        _, buffer = cv2.imencode('.jpg', self.image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buffer.tobytes()


# Callback type for frame processing
# Can return modified Frame or None to keep original
FrameCallback = Callable[[Frame], Optional[Frame]]
