"""
Web streaming support for drone video.

Provides MJPEG streaming over HTTP for browser viewing.
Can be used standalone or as a frame callback.

Requires Flask (optional dependency):
```bash
pip install flask
```
"""

import threading
import time
from typing import Optional, Generator, Callable
from collections import deque
from queue import Queue, Empty

from .stream import VideoStream
from .types import Frame, FrameCallback


class MJPEGStreamer:
    """
    MJPEG streamer as a frame callback.

    Buffers frames and provides a generator for HTTP streaming.
    Compatible with Flask, FastAPI, or any WSGI/ASGI framework.

    Example with Flask:
    ```python
    from flask import Flask, Response
    from pyhulax.video import VideoStream, MJPEGStreamer

    app = Flask(__name__)
    streamer = MJPEGStreamer()

    # Add to video stream
    stream = VideoStream()
    stream.add_callback(streamer)
    stream.start()

    @app.route('/video')
    def video_feed():
        return Response(
            streamer.generate(),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    app.run(host='0.0.0.0', port=5000)
    ```
    """

    def __init__(
        self,
        quality: int = 80,
        max_fps: float = 30.0,
        buffer_size: int = 2,
        draw_detections: bool = True,
    ):
        """
        Initialize MJPEG streamer.

        Args:
            quality: JPEG quality (1-100)
            max_fps: Maximum frames per second to stream
            buffer_size: Frame buffer size
            draw_detections: Draw detection boxes on streamed frames
        """
        self.quality = quality
        self.max_fps = max_fps
        self.draw_detections = draw_detections

        self._frame_interval = 1.0 / max_fps
        self._last_frame_time = 0.0

        # Thread-safe frame buffer
        self._buffer: deque = deque(maxlen=buffer_size)
        self._lock = threading.Lock()

        # Latest JPEG bytes
        self._latest_jpeg: Optional[bytes] = None

        # Stats
        self._frame_count = 0
        self._client_count = 0

    @property
    def frame_count(self) -> int:
        """Number of frames processed."""
        return self._frame_count

    @property
    def client_count(self) -> int:
        """Number of active streaming clients."""
        return self._client_count

    def __call__(self, frame: Frame) -> Frame:
        """
        Process frame (callback interface).

        Encodes frame to JPEG and buffers for streaming.

        Args:
            frame: Input frame

        Returns:
            Same frame (unmodified)
        """
        now = time.time()

        # Rate limit
        if now - self._last_frame_time < self._frame_interval:
            return frame

        self._last_frame_time = now

        # Get image (with or without detections)
        if self.draw_detections and frame.detections:
            img = frame.draw_detections()
        else:
            img = frame.image

        # Encode to JPEG
        import cv2
        _, jpeg = cv2.imencode(
            '.jpg', img,
            [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        )
        jpeg_bytes = jpeg.tobytes()

        # Update buffer
        with self._lock:
            self._buffer.append(jpeg_bytes)
            self._latest_jpeg = jpeg_bytes

        self._frame_count += 1

        return frame

    def get_frame(self) -> Optional[bytes]:
        """
        Get latest JPEG frame.

        Returns:
            JPEG bytes or None if no frame available
        """
        with self._lock:
            return self._latest_jpeg

    def generate(self) -> Generator[bytes, None, None]:
        """
        Generate MJPEG stream for HTTP response.

        Yields:
            MJPEG frame bytes with boundary markers
        """
        self._client_count += 1
        last_frame = None

        try:
            while True:
                # Get frame
                with self._lock:
                    jpeg = self._latest_jpeg

                # Skip if same frame
                if jpeg is None or jpeg is last_frame:
                    time.sleep(0.01)
                    continue

                last_frame = jpeg

                # Yield MJPEG frame
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    jpeg +
                    b'\r\n'
                )

        finally:
            self._client_count -= 1


class WebStreamServer:
    """
    Standalone web server for video streaming.

    Provides a simple Flask-based server for viewing the drone
    video stream in a web browser.

    Example:
    ```python
    from pyhulax.video import VideoStream, WebStreamServer

    stream = VideoStream()

    # Start web server (runs in background)
    server = WebStreamServer(stream, port=5000)
    server.start()

    print("Open http://localhost:5000 in browser")

    stream.start(blocking=True)
    ```
    """

    def __init__(
        self,
        video_stream: VideoStream = None,
        host: str = "0.0.0.0",
        port: int = 5000,
        quality: int = 80,
        draw_detections: bool = True,
    ):
        """
        Initialize web server.

        Args:
            video_stream: VideoStream instance to serve
            host: Server host
            port: Server port
            quality: JPEG quality
            draw_detections: Draw detections on stream
        """
        self.host = host
        self.port = port

        self._streamer = MJPEGStreamer(
            quality=quality,
            draw_detections=draw_detections,
        )

        self._video_stream = video_stream
        self._thread: Optional[threading.Thread] = None
        self._app = None

        # Add streamer to video stream callbacks
        if video_stream is not None:
            video_stream.add_callback(self._streamer)

    def _create_app(self):
        """Create Flask application."""
        try:
            from flask import Flask, Response, render_template_string
        except ImportError:
            raise ImportError(
                "Flask is required for web streaming. "
                "Install with: pip install flask"
            )

        app = Flask(__name__)

        # HTML template for viewer page
        HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Drone Video Stream</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
            font-family: system-ui, sans-serif;
        }
        h1 { margin-bottom: 10px; }
        .container {
            max-width: 1280px;
            margin: 0 auto;
        }
        .video-container {
            background: #000;
            border-radius: 8px;
            overflow: hidden;
            position: relative;
        }
        img {
            width: 100%;
            height: auto;
            display: block;
        }
        .stats {
            margin-top: 10px;
            font-size: 14px;
            color: #888;
        }
        .controls {
            margin-top: 20px;
        }
        button {
            background: #333;
            color: #fff;
            border: none;
            padding: 10px 20px;
            border-radius: 4px;
            cursor: pointer;
            margin-right: 10px;
        }
        button:hover { background: #444; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Drone Video Stream</h1>
        <div class="video-container">
            <img src="/video_feed" alt="Video stream">
        </div>
        <div class="stats">
            Stream: <span id="status">Connected</span>
        </div>
        <div class="controls">
            <button onclick="location.reload()">Refresh</button>
            <button onclick="toggleFullscreen()">Fullscreen</button>
        </div>
    </div>
    <script>
        function toggleFullscreen() {
            const img = document.querySelector('img');
            if (img.requestFullscreen) {
                img.requestFullscreen();
            }
        }
    </script>
</body>
</html>
'''

        @app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @app.route('/video_feed')
        def video_feed():
            return Response(
                self._streamer.generate(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @app.route('/frame.jpg')
        def single_frame():
            """Get single JPEG frame."""
            jpeg = self._streamer.get_frame()
            if jpeg is None:
                return Response(status=204)  # No content
            return Response(jpeg, mimetype='image/jpeg')

        return app

    def start(self, blocking: bool = False) -> None:
        """
        Start the web server.

        Args:
            blocking: If True, block until server stops
        """
        self._app = self._create_app()

        if blocking:
            self._app.run(host=self.host, port=self.port, threaded=True)
        else:
            self._thread = threading.Thread(
                target=self._app.run,
                kwargs={
                    'host': self.host,
                    'port': self.port,
                    'threaded': True,
                    'use_reloader': False,
                },
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        """Stop the web server."""
        # Flask doesn't have a clean shutdown from another thread
        # The daemon thread will stop when the main program exits
        pass

    @property
    def url(self) -> str:
        """Server URL."""
        host = 'localhost' if self.host == '0.0.0.0' else self.host
        return f"http://{host}:{self.port}"

    @property
    def streamer(self) -> MJPEGStreamer:
        """Get the MJPEG streamer instance."""
        return self._streamer
