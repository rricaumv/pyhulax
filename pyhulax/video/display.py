"""
OpenCV-based video display for drone stream.
"""

import cv2
import numpy as np
import threading
import time
from typing import Optional,  Callable

from .types import Frame


class VideoDisplay:
    """
    OpenCV window display as a frame callback.

    Displays frames with optional detection overlays, FPS counter,
    and recording indicator.

    Example:
    ```python
    stream = VideoStream(drone_ip="192.168.100.1")
    display = VideoDisplay(window_name="Drone", show_fps=True)
    stream.add_callback(display)
    stream.start()

    # Display handles its own window events
    # Press 'q' to quit, 's' to screenshot
    ```
    """

    def __init__(
        self,
        window_name: str = "Drone Video",
        show_fps: bool = True,
        show_detections: bool = True,
        show_info: bool = True,
        scale: float = 1.0,
        fullscreen: bool = False,
        on_key: Optional[Callable[[int], None]] = None,
    ):
        """
        Initialize display.

        Args:
            window_name: OpenCV window name
            show_fps: Show FPS counter
            show_detections: Draw detection bounding boxes
            show_info: Show frame info overlay
            scale: Display scale factor (1.0 = original size)
            fullscreen: Start in fullscreen mode
            on_key: Callback for key presses (receives key code)
        """
        self.window_name = window_name
        self.show_fps = show_fps
        self.show_detections = show_detections
        self.show_info = show_info
        self.scale = scale
        self.fullscreen = fullscreen
        self.on_key = on_key

        self._window_created = False
        self._fps_times: list = []
        self._fps = 0.0
        self._frame_count = 0
        self._recording = False
        self._should_stop = False

        # Screenshot path
        self._screenshot_dir = "."
        self._last_screenshot: Optional[str] = None

    @property
    def fps(self) -> float:
        """Current display FPS."""
        return self._fps

    @property
    def should_stop(self) -> bool:
        """Check if user requested stop (pressed 'q')."""
        return self._should_stop

    def set_recording(self, recording: bool) -> None:
        """Set recording indicator state."""
        self._recording = recording

    def _create_window(self) -> None:
        """Create OpenCV window."""
        if self._window_created:
            return

        if self.fullscreen:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(
                self.window_name,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN
            )
        else:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        self._window_created = True

    def _update_fps(self) -> None:
        """Update FPS calculation."""
        now = time.time()
        self._fps_times.append(now)

        # Keep last 30 timestamps
        while len(self._fps_times) > 30:
            self._fps_times.pop(0)

        if len(self._fps_times) >= 2:
            elapsed = self._fps_times[-1] - self._fps_times[0]
            if elapsed > 0:
                self._fps = (len(self._fps_times) - 1) / elapsed

    def _draw_overlay(self, image: np.ndarray, frame: Frame) -> np.ndarray:
        """Draw info overlay on image."""
        h, w = image.shape[:2]

        # FPS counter (top-left)
        if self.show_fps:
            fps_text = f"FPS: {self._fps:.1f}"
            cv2.putText(
                image, fps_text, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

        # Frame info (top-left, below FPS)
        if self.show_info:
            info_text = f"Frame: {frame.frame_number} | {w}x{h}"
            if frame.detections:
                info_text += f" | {len(frame.detections)} detections"
            cv2.putText(
                image, info_text, (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )

        # Recording indicator (top-right)
        if self._recording:
            cv2.circle(image, (w - 20, 20), 10, (0, 0, 255), -1)
            cv2.putText(
                image, "REC", (w - 60, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )

        # Detection count (bottom-left)
        if frame.detections:
            for i, det in enumerate(frame.detections[:5]):  # Show up to 5
                y = h - 20 - (i * 20)
                det_text = f"{det.label}: {det.confidence:.2f}"
                cv2.putText(
                    image, det_text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, det.color, 1
                )

        return image

    def _handle_key(self, key: int, frame: Frame) -> None:
        """Handle keyboard input."""
        if key == -1:
            return

        key = key & 0xFF

        # Quit
        if key == ord('q') or key == 27:  # 'q' or ESC
            self._should_stop = True

        # Screenshot
        elif key == ord('s'):
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            path = f"{self._screenshot_dir}/drone_screenshot_{timestamp}.jpg"
            cv2.imwrite(path, frame.image)
            self._last_screenshot = path
            print(f"Screenshot saved: {path}")

        # Toggle fullscreen
        elif key == ord('f'):
            self.fullscreen = not self.fullscreen
            if self.fullscreen:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN
                )
            else:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_NORMAL
                )

        # Toggle detection display
        elif key == ord('d'):
            self.show_detections = not self.show_detections

        # Toggle info display
        elif key == ord('i'):
            self.show_info = not self.show_info

        # User callback
        if self.on_key:
            self.on_key(key)

    def __call__(self, frame: Frame) -> Frame:
        """
        Process and display frame.

        This is the callback interface for VideoStream.

        Args:
            frame: Input frame

        Returns:
            Same frame (unmodified)
        """
        self._create_window()
        self._update_fps()
        self._frame_count += 1

        # Get display image
        if self.show_detections and frame.detections:
            display_img = frame.draw_detections()
        else:
            display_img = frame.image.copy()

        # Draw overlay
        display_img = self._draw_overlay(display_img, frame)

        # Scale if needed
        if self.scale != 1.0:
            h, w = display_img.shape[:2]
            new_w = int(w * self.scale)
            new_h = int(h * self.scale)
            display_img = cv2.resize(display_img, (new_w, new_h))

        # Show
        cv2.imshow(self.window_name, display_img)

        # Handle input (1ms wait for responsiveness)
        key = cv2.waitKey(1)
        self._handle_key(key, frame)

        return frame

    def close(self) -> None:
        """Close the display window."""
        if self._window_created:
            cv2.destroyWindow(self.window_name)
            self._window_created = False

    def __del__(self):
        """Cleanup on deletion."""
        self.close()


class VideoDisplayAsync:
    """
    Async display that runs in its own thread.

    Useful when you want display to run independently
    of the frame processing pipeline.

    Example:
    ```python
    display = VideoDisplayAsync()
    display.start()

    for frame in stream:
        detections = detector.detect(frame)
        frame.detections = detections
        display.update(frame)  # Non-blocking

    display.stop()
    ```
    """

    def __init__(self, **kwargs):
        """
        Initialize async display.

        Args:
            **kwargs: Arguments passed to VideoDisplay
        """
        self._display = VideoDisplay(**kwargs)
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame: Optional[Frame] = None
        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()

    def start(self) -> None:
        """Start display thread."""
        if self._thread is not None:
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._display_loop, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        """Stop display thread."""
        self._stop_event.set()
        self._frame_ready.set()  # Wake up thread

        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

        self._display.close()

    def update(self, frame: Frame) -> None:
        """
        Update displayed frame (non-blocking).

        Args:
            frame: New frame to display
        """
        with self._frame_lock:
            self._frame = frame
        self._frame_ready.set()

    @property
    def should_stop(self) -> bool:
        """Check if user requested stop."""
        return self._display.should_stop

    def _display_loop(self) -> None:
        """Display thread main loop."""
        while not self._stop_event.is_set():
            # Wait for frame
            if not self._frame_ready.wait(timeout=0.1):
                continue

            self._frame_ready.clear()

            # Get frame
            with self._frame_lock:
                frame = self._frame
                self._frame = None

            if frame is None:
                continue

            # Display
            self._display(frame)

            if self._display.should_stop:
                break


def show_frame(
    frame: Frame,
    window_name: str = "Frame",
    wait_key: int = 0,
    show_detections: bool = True,
) -> int:
    """
    Quick utility to show a single frame.

    Args:
        frame: Frame to display
        window_name: Window name
        wait_key: cv2.waitKey argument (0 = wait forever)
        show_detections: Draw detections if present

    Returns:
        Key code pressed
    """
    if show_detections and frame.detections:
        img = frame.draw_detections()
    else:
        img = frame.image

    cv2.imshow(window_name, img)
    return cv2.waitKey(wait_key)
