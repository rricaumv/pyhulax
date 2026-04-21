"""
Video recording functionality using PyAV.

Provides callback-based recording for the VideoStream pipeline.
"""

import av
import threading
import time
from fractions import Fraction
from pathlib import Path
from typing import Optional, Union
from dataclasses import dataclass, field

from .types import Frame


@dataclass
class RecordingConfig:
    """Configuration for video recording."""
    output_path: str
    codec: str = "libx264"
    fps: float = 30.0
    bitrate: Optional[int] = None  # e.g., 2_000_000 for 2Mbps
    pix_fmt: str = "yuv420p"
    preset: str = "fast"  # ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow
    crf: int = 23  # Constant Rate Factor (0-51, lower = better quality)

    # Container format (auto-detected from extension if None)
    container_format: Optional[str] = None


class VideoRecorder:
    """
    Video recording callback for VideoStream.

    Records frames to a video file using PyAV (FFmpeg).
    Can be used as a callback in the VideoStream pipeline.

    Example:
    ```python
    stream = VideoStream(drone_ip="192.168.100.1")

    # Basic recording
    recorder = VideoRecorder("flight.mp4")
    stream.add_callback(recorder)
    stream.start()
    # ... stream video ...
    stream.stop()
    recorder.close()

    # With context manager
    with VideoRecorder("flight.mp4") as recorder:
        stream.add_callback(recorder)
        stream.start()
        stream.wait()
    # File automatically finalized

    # Record with detections drawn
    recorder = VideoRecorder("flight_annotated.mp4", draw_detections=True)
    ```
    """

    def __init__(
        self,
        output_path: Union[str, Path],
        fps: float = 30.0,
        codec: str = "libx264",
        bitrate: Optional[int] = None,
        preset: str = "fast",
        crf: int = 23,
        draw_detections: bool = False,
    ):
        """
        Initialize video recorder.

        Args:
            output_path: Output video file path (.mp4, .avi, .mkv, etc.)
            fps: Frame rate for output video
            codec: Video codec (libx264, h264_nvenc for NVIDIA, etc.)
            bitrate: Optional bitrate in bits/sec (uses CRF if None)
            preset: x264 preset (ultrafast to veryslow)
            crf: Constant Rate Factor for quality (0-51, lower=better)
            draw_detections: If True, draw detection boxes on recorded frames
        """
        self.config = RecordingConfig(
            output_path=str(output_path),
            codec=codec,
            fps=fps,
            bitrate=bitrate,
            preset=preset,
            crf=crf,
        )

        self._draw_detections = draw_detections
        self._container: Optional[av.container.OutputContainer] = None
        self._stream: Optional[av.stream.Stream] = None
        self._initialized = False
        self._closed = False
        self._frame_count = 0
        self._start_time: Optional[float] = None
        self._lock = threading.Lock()

    @property
    def frame_count(self) -> int:
        """Number of frames recorded."""
        return self._frame_count

    @property
    def duration(self) -> float:
        """Recording duration in seconds."""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    @property
    def is_recording(self) -> bool:
        """Check if recorder is active."""
        return self._initialized and not self._closed

    def _init_output(self, width: int, height: int) -> None:
        """Initialize output container and stream."""
        self._container = av.open(self.config.output_path, mode='w')

        # Add video stream (rate must be a Fraction for PyAV)
        fps_fraction = Fraction(self.config.fps).limit_denominator(1000)
        self._stream = self._container.add_stream(self.config.codec, rate=fps_fraction)
        self._stream.width = width
        self._stream.height = height
        self._stream.pix_fmt = self.config.pix_fmt

        # Set encoding options
        if self.config.bitrate:
            self._stream.bit_rate = self.config.bitrate
        else:
            # Use CRF mode
            self._stream.options = {
                'crf': str(self.config.crf),
                'preset': self.config.preset,
            }

        self._initialized = True
        self._start_time = time.time()

    def __call__(self, frame: Frame) -> Frame:
        """
        Process frame callback - records frame to file.

        Args:
            frame: Input frame from VideoStream

        Returns:
            Unmodified frame (passthrough for pipeline)
        """
        if self._closed:
            return frame

        with self._lock:
            # Initialize on first frame (gets dimensions)
            if not self._initialized:
                self._init_output(frame.width, frame.height)

            # Get image to record
            if self._draw_detections and frame.detections:
                image = frame.draw_detections()
            else:
                image = frame.image

            # Convert numpy array to av.VideoFrame
            av_frame = av.VideoFrame.from_ndarray(image, format='bgr24')
            av_frame.pts = self._frame_count

            # Encode and write
            for packet in self._stream.encode(av_frame):
                self._container.mux(packet)

            self._frame_count += 1

        return frame

    def write_frame(self, frame: Frame) -> None:
        """
        Manually write a frame (alternative to callback usage).

        Args:
            frame: Frame to record
        """
        self(frame)

    def close(self) -> None:
        """
        Finalize and close the video file.

        Must be called when recording is complete to flush
        remaining frames and write file trailer.
        """
        if self._closed:
            return

        with self._lock:
            self._closed = True

            if self._stream is not None:
                # Flush encoder
                for packet in self._stream.encode():
                    self._container.mux(packet)

            if self._container is not None:
                self._container.close()
                self._container = None

    def __enter__(self) -> "VideoRecorder":
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit - ensures file is finalized."""
        self.close()

    def __del__(self):
        """Destructor - attempt to close if not already closed."""
        if not self._closed:
            try:
                self.close()
            except Exception:
                pass


class SegmentedRecorder:
    """
    Records video in segments of fixed duration.

    Useful for long recordings or when you want multiple
    smaller files instead of one large file.

    Example:
    ```python
    # Record in 5-minute segments
    recorder = SegmentedRecorder(
        output_dir="recordings",
        segment_duration=300,  # 5 minutes
        filename_pattern="flight_{timestamp}_{segment}.mp4"
    )
    stream.add_callback(recorder)
    ```
    """

    def __init__(
        self,
        output_dir: Union[str, Path],
        segment_duration: float = 300.0,
        filename_pattern: str = "segment_{timestamp}_{segment:04d}.mp4",
        **recorder_kwargs,
    ):
        """
        Initialize segmented recorder.

        Args:
            output_dir: Directory to save segment files
            segment_duration: Duration of each segment in seconds
            filename_pattern: Filename pattern with {timestamp} and {segment} placeholders
            **recorder_kwargs: Additional arguments passed to VideoRecorder
        """
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._segment_duration = segment_duration
        self._filename_pattern = filename_pattern
        self._recorder_kwargs = recorder_kwargs

        self._current_recorder: Optional[VideoRecorder] = None
        self._segment_index = 0
        self._segment_start_time: Optional[float] = None
        self._session_timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._lock = threading.Lock()

    def _get_segment_path(self) -> Path:
        """Generate path for current segment."""
        filename = self._filename_pattern.format(
            timestamp=self._session_timestamp,
            segment=self._segment_index,
        )
        return self._output_dir / filename

    def _start_new_segment(self) -> None:
        """Start a new recording segment."""
        # Close current segment if exists
        if self._current_recorder is not None:
            self._current_recorder.close()

        # Create new recorder
        self._current_recorder = VideoRecorder(
            self._get_segment_path(),
            **self._recorder_kwargs,
        )
        self._segment_start_time = time.time()
        self._segment_index += 1

    def __call__(self, frame: Frame) -> Frame:
        """Process frame callback."""
        with self._lock:
            # Start first segment or check if we need a new one
            if self._current_recorder is None:
                self._start_new_segment()
            elif time.time() - self._segment_start_time >= self._segment_duration:
                self._start_new_segment()

            # Record to current segment
            return self._current_recorder(frame)

    def close(self) -> None:
        """Close current segment and finalize."""
        with self._lock:
            if self._current_recorder is not None:
                self._current_recorder.close()
                self._current_recorder = None

    def __enter__(self) -> "SegmentedRecorder":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
