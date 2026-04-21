"""
Video stream decoder using PyAV for RTP/RTSP H.264 streams.
"""

import av
import cv2
import numpy as np
import socket
import tempfile
import threading
import time
import os
from collections import deque
from typing import Callable, List, Optional, Deque, Union
from queue import Queue, Empty

from pyhulax.config import DroneConfig, resolve_config
from .types import Frame, FrameCallback, StreamState, StreamConfig


class RTSPStream:
    """
    RTSP video stream decoder using PyAV.

    For testing without a drone or connecting to any RTSP source.

    Example:
    ```python
    stream = RTSPStream("rtsp://localhost:8554/stream")
    def detect(frame):
        frame.detections = model.detect(frame.image)
        return frame

    stream.add_callback(detect)
    stream.add_callback(VideoDisplay())
    stream.start()
    stream.wait()
    ```
    """

    def __init__(
        self,
        url: str,
        timeout: float = 10.0,
        buffer_size: int = 10,
        tcp_transport: bool = True,
    ):
        """
        Initialize RTSP stream.

        Args:
            url: RTSP URL (e.g., "rtsp://localhost:8554/stream")
            timeout: Connection timeout in seconds
            buffer_size: Frame buffer size
            tcp_transport: Use TCP for RTP (more reliable, default True)
        """
        self._url = url
        self._timeout = timeout
        self._tcp_transport = tcp_transport

        self._state = StreamState.DISCONNECTED
        self._callbacks: List[FrameCallback] = []
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = 0.0
        self._fps_frame_count = 0

        self._frame_buffer: Deque[Frame] = deque(maxlen=buffer_size)
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[Frame] = None
        self._last_error: Optional[str] = None

    @property
    def url(self) -> str:
        """RTSP URL."""
        return self._url

    @property
    def state(self) -> StreamState:
        """Current stream state."""
        return self._state

    @property
    def is_streaming(self) -> bool:
        """Check if stream is actively running."""
        return self._state == StreamState.STREAMING

    @property
    def fps(self) -> float:
        """Current frames per second."""
        return self._fps

    @property
    def frame_count(self) -> int:
        """Total frames decoded."""
        return self._frame_count

    @property
    def latest_frame(self) -> Optional[Frame]:
        """Get the most recent frame (thread-safe)."""
        with self._frame_lock:
            return self._latest_frame

    @property
    def last_error(self) -> Optional[str]:
        """Last error message, if any."""
        return self._last_error

    def add_callback(self, callback: FrameCallback) -> None:
        """Add a frame processing callback."""
        self._callbacks.append(callback)

    def remove_callback(self, callback: FrameCallback) -> bool:
        """Remove a callback."""
        try:
            self._callbacks.remove(callback)
            return True
        except ValueError:
            return False

    def clear_callbacks(self) -> None:
        """Remove all callbacks."""
        self._callbacks.clear()

    def get_buffered_frames(self, count: Optional[int] = None) -> List[Frame]:
        """Get frames from the buffer."""
        with self._frame_lock:
            if count is None:
                return list(self._frame_buffer)
            return list(self._frame_buffer)[-count:]

    def _decode_loop(self) -> None:
        """Main decode loop (runs in thread)."""
        container = None

        try:
            self._state = StreamState.CONNECTING

            # PyAV options for RTSP
            options = {
                'rtsp_transport': 'tcp' if self._tcp_transport else 'udp',
                'fflags': 'nobuffer',
                'flags': 'low_delay',
            }

            container = av.open(
                self._url,
                options=options,
                timeout=self._timeout,
            )

            self._state = StreamState.CONNECTED

            # Find video stream
            video_stream = None
            for stream in container.streams:
                if stream.type == 'video':
                    video_stream = stream
                    break

            if video_stream is None:
                raise RuntimeError("No video stream found")

            video_stream.thread_type = 'AUTO'

            self._state = StreamState.STREAMING
            self._last_fps_time = time.time()

            for packet in container.demux(video_stream):
                if self._stop_event.is_set():
                    break

                for av_frame in packet.decode():
                    if self._stop_event.is_set():
                        break

                    img = av_frame.to_ndarray(format='bgr24')

                    frame = Frame(
                        image=img,
                        frame_number=self._frame_count,
                    )

                    self._frame_count += 1

                    # Run callbacks
                    for callback in self._callbacks:
                        try:
                            result = callback(frame)
                            if result is not None:
                                frame = result
                        except Exception as e:
                            print(f"Callback error: {e}")

                    # Update buffer
                    with self._frame_lock:
                        self._frame_buffer.append(frame)
                        self._latest_frame = frame

                    # Update FPS
                    self._fps_frame_count += 1
                    now = time.time()
                    elapsed = now - self._last_fps_time
                    if elapsed >= 1.0:
                        self._fps = self._fps_frame_count / elapsed
                        self._fps_frame_count = 0
                        self._last_fps_time = now

        except av.error.ExitError:
            pass
        except Exception as e:
            self._last_error = str(e)
            self._state = StreamState.ERROR
            print(f"Stream error: {e}")
        finally:
            if container:
                container.close()

            if self._state != StreamState.ERROR:
                self._state = StreamState.STOPPED

    def start(self, blocking: bool = False) -> None:
        """Start the video stream."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("Stream already running")

        self._stop_event.clear()
        self._frame_count = 0
        self._fps = 0.0
        self._last_error = None

        self._thread = threading.Thread(target=self._decode_loop, daemon=True)
        self._thread.start()

        if blocking:
            self.wait()

    def stop(self, timeout: float = 5.0) -> None:
        """Stop the video stream."""
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

        self._state = StreamState.STOPPED

    def wait(self) -> None:
        """Block until stream stops."""
        if self._thread is not None:
            self._thread.join()

    def __enter__(self) -> "RTSPStream":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()


class VideoStream:
    """
    RTP H.264 video stream decoder using PyAV.

    Decodes drone video stream and passes frames through a callback pipeline.
    Supports multiple callbacks for detection, display, recording, etc.

    Example:
    ```python
    stream = VideoStream(drone_ip="192.168.100.1")

    # Add detection callback
    def detect(frame):
        frame.detections = my_model.detect(frame.image)
        return frame
    stream.add_callback(detect)

    # Add display
    stream.add_callback(display)

    stream.start()
    stream.wait()  # Block until stopped
    ```
    """

    def __init__(
        self,
        drone_ip: str | None = None,
        drone_id: int = 1,
        timeout: float | None = None,
        buffer_size: int | None = None,
        config: DroneConfig | None = None,
    ):
        """
        Initialize video stream.

        Args:
            drone_ip: Drone IP address
            drone_id: Drone ID (determines RTP port: 9000 + drone_id * 2)
            timeout: Connection timeout in seconds
            buffer_size: Frame buffer size for web streaming
        """
        runtime_config = resolve_config(config)
        self.config = StreamConfig(
            drone_ip=drone_ip or runtime_config.network.drone_ip,
            drone_id=drone_id,
            timeout=timeout if timeout is not None else runtime_config.video.timeout_sec,
            buffer_size=(
                buffer_size
                if buffer_size is not None
                else runtime_config.video.buffer_size
            ),
        )

        self._state = StreamState.DISCONNECTED
        self._callbacks: List[FrameCallback] = []
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = 0.0
        self._fps_frame_count = 0

        # Frame buffer for web streaming (thread-safe deque)
        self._frame_buffer: Deque[Frame] = deque(maxlen=self.config.buffer_size)
        self._frame_lock = threading.Lock()

        # Latest frame for quick access
        self._latest_frame: Optional[Frame] = None

        # Error tracking
        self._last_error: Optional[str] = None

        # SDP file path (temp file)
        self._sdp_path: Optional[str] = None

    @property
    def state(self) -> StreamState:
        """Current stream state."""
        return self._state

    @property
    def is_streaming(self) -> bool:
        """Check if stream is actively running."""
        return self._state == StreamState.STREAMING

    @property
    def fps(self) -> float:
        """Current frames per second."""
        return self._fps

    @property
    def frame_count(self) -> int:
        """Total frames decoded."""
        return self._frame_count

    @property
    def latest_frame(self) -> Optional[Frame]:
        """Get the most recent frame (thread-safe)."""
        with self._frame_lock:
            return self._latest_frame

    @property
    def last_error(self) -> Optional[str]:
        """Last error message, if any."""
        return self._last_error

    def add_callback(self, callback: FrameCallback) -> None:
        """
        Add a frame processing callback.

        Callbacks are executed in order. Each receives the frame
        (potentially modified by previous callbacks) and can:

        - Add detections
        - Modify the image
        - Return modified Frame or None

        Args:
            callback: Function taking Frame, returning Frame or None
        """
        self._callbacks.append(callback)

    def remove_callback(self, callback: FrameCallback) -> bool:
        """
        Remove a callback.

        Args:
            callback: Callback to remove

        Returns:
            True if removed, False if not found
        """
        try:
            self._callbacks.remove(callback)
            return True
        except ValueError:
            return False

    def clear_callbacks(self) -> None:
        """Remove all callbacks."""
        self._callbacks.clear()

    def get_buffered_frames(self, count: Optional[int] = None) -> List[Frame]:
        """
        Get frames from the buffer.

        Args:
            count: Number of frames to get (None for all)

        Returns:
            List of frames (oldest first)
        """
        with self._frame_lock:
            if count is None:
                return list(self._frame_buffer)
            return list(self._frame_buffer)[-count:]

    def _get_local_ip(self) -> str:
        """Get local IP address for drone network."""
        try:
            # Connect to drone to find which interface we're using
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((self.config.drone_ip, 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            # Fallback to common drone network IP
            return "192.168.100.2"

    def _create_sdp_file(self) -> str:
        """Create temporary SDP file for PyAV."""
        local_ip = self._get_local_ip()
        sdp_content = self.config.generate_sdp(local_ip)

        # Create temp file
        fd, path = tempfile.mkstemp(suffix=".sdp", prefix="drone_video_")
        with os.fdopen(fd, 'w') as f:
            f.write(sdp_content)

        self._sdp_path = path
        return path

    def _cleanup_sdp_file(self) -> None:
        """Remove temporary SDP file."""
        if self._sdp_path and os.path.exists(self._sdp_path):
            try:
                os.unlink(self._sdp_path)
            except Exception:
                pass
            self._sdp_path = None

    def _decode_loop(self) -> None:
        """Main decode loop (runs in thread)."""
        container = None

        try:
            self._state = StreamState.CONNECTING

            # Create SDP file
            sdp_path = self._create_sdp_file()

            # PyAV options for RTP (matching working taskcontroller settings)
            options = {
                'protocol_whitelist': 'file,rtp,udp',
                'fflags': 'nobuffer',
                'flags': 'low_delay',
                'reorder_queue_size': '0',
                'max_delay': '0',
            }

            # Open stream
            container = av.open(
                sdp_path,
                format='sdp',
                options=options,
                timeout=self.config.timeout,
            )

            self._state = StreamState.CONNECTED

            # Find video stream
            video_stream = None
            for stream in container.streams:
                if stream.type == 'video':
                    video_stream = stream
                    break

            if video_stream is None:
                raise RuntimeError("No video stream found")

            # Configure decoder for low latency
            video_stream.thread_type = 'AUTO'

            self._state = StreamState.STREAMING
            self._last_fps_time = time.time()

            # Decode frames
            for packet in container.demux(video_stream):
                if self._stop_event.is_set():
                    break

                for av_frame in packet.decode():
                    if self._stop_event.is_set():
                        break

                    # Convert to numpy BGR
                    img = av_frame.to_ndarray(format='bgr24')

                    # Create Frame object
                    frame = Frame(
                        image=img,
                        frame_number=self._frame_count,
                    )

                    self._frame_count += 1

                    # Run through callback pipeline
                    for callback in self._callbacks:
                        try:
                            result = callback(frame)
                            if result is not None:
                                frame = result
                        except Exception as e:
                            print(f"Callback error: {e}")

                    # Update buffer
                    with self._frame_lock:
                        self._frame_buffer.append(frame)
                        self._latest_frame = frame

                    # Update FPS
                    self._fps_frame_count += 1
                    now = time.time()
                    elapsed = now - self._last_fps_time
                    if elapsed >= 1.0:
                        self._fps = self._fps_frame_count / elapsed
                        self._fps_frame_count = 0
                        self._last_fps_time = now

        except av.error.ExitError:
            # Normal exit
            pass
        except Exception as e:
            self._last_error = str(e)
            self._state = StreamState.ERROR
            print(f"Stream error: {e}")
        finally:
            if container:
                container.close()
            self._cleanup_sdp_file()

            if self._state != StreamState.ERROR:
                self._state = StreamState.STOPPED

    def start(self, blocking: bool = False) -> None:
        """
        Start the video stream.

        Args:
            blocking: If True, block until stream stops
        """
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("Stream already running")

        self._stop_event.clear()
        self._frame_count = 0
        self._fps = 0.0
        self._last_error = None

        self._thread = threading.Thread(target=self._decode_loop, daemon=True)
        self._thread.start()

        if blocking:
            self.wait()

    def stop(self, timeout: float = 5.0) -> None:
        """
        Stop the video stream.

        Args:
            timeout: Seconds to wait for thread to stop
        """
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

        self._state = StreamState.STOPPED

    def wait(self) -> None:
        """Block until stream stops."""
        if self._thread is not None:
            self._thread.join()

    def __enter__(self) -> "VideoStream":
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.stop()


class VideoStreamSimple:
    """
    Simplified video stream for quick testing.

    Opens stream and yields frames directly without callbacks.

    Example:
    ```python
    for frame in VideoStreamSimple("192.168.100.1"):
        cv2.imshow("Video", frame.image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    ```
    """

    def __init__(
        self,
        drone_ip: str | None = None,
        drone_id: int = 1,
        timeout: float | None = None,
        config: DroneConfig | None = None,
    ):
        runtime_config = resolve_config(config)
        self.config = StreamConfig(
            drone_ip=drone_ip or runtime_config.network.drone_ip,
            drone_id=drone_id,
            timeout=timeout if timeout is not None else runtime_config.video.timeout_sec,
        )
        self._container = None
        self._sdp_path: Optional[str] = None

    def _get_local_ip(self) -> str:
        """Get local IP address for drone network."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((self.config.drone_ip, 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            return "192.168.100.2"

    def __iter__(self):
        """Iterate over frames."""
        # Create SDP file
        local_ip = self._get_local_ip()
        sdp_content = self.config.generate_sdp(local_ip)

        fd, self._sdp_path = tempfile.mkstemp(suffix=".sdp")
        with os.fdopen(fd, 'w') as f:
            f.write(sdp_content)

        try:
            options = {
                'protocol_whitelist': 'file,rtp,udp',
                'fflags': 'nobuffer',
                'flags': 'low_delay',
            }

            self._container = av.open(
                self._sdp_path,
                format='sdp',
                options=options,
                timeout=self.config.timeout,
            )

            video_stream = None
            for stream in self._container.streams:
                if stream.type == 'video':
                    video_stream = stream
                    break

            if video_stream is None:
                raise RuntimeError("No video stream found")

            frame_number = 0
            for packet in self._container.demux(video_stream):
                for av_frame in packet.decode():
                    img = av_frame.to_ndarray(format='bgr24')
                    yield Frame(image=img, frame_number=frame_number)
                    frame_number += 1

        finally:
            self.close()

    def close(self) -> None:
        """Close stream and cleanup."""
        if self._container:
            self._container.close()
            self._container = None

        if self._sdp_path and os.path.exists(self._sdp_path):
            try:
                os.unlink(self._sdp_path)
            except Exception:
                pass
            self._sdp_path = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
