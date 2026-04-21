# Video and Logging

The SDK ships video and logging support as optional layers around the core API.

## Optional Extras

Install what you need:

```bash
pip install "pyhulax[video]"
pip install "pyhulax[vision]"
pip install "pyhulax[web]"
pip install "pyhulax[db]"
```

## [`pyhulax.video`][pyhulax.video]

[`pyhulax.video`][pyhulax.video] lazy-loads optional backends so importing [`pyhulax`][pyhulax.DroneAPI] does not require OpenCV, PyAV, ONNX Runtime, or Flask.

Core exported types:

- [`Frame`][pyhulax.video.types.Frame]
- [`Detection`][pyhulax.video.types.Detection]
- [`BoundingBox`][pyhulax.video.types.BoundingBox]
- [`FrameCallback`][pyhulax.video.types.FrameCallback]
- [`StreamConfig`][pyhulax.video.types.StreamConfig]
- [`StreamState`][pyhulax.video.types.StreamState]

Streaming classes:

- [`VideoStream`][pyhulax.video.stream.VideoStream]
- [`VideoStreamSimple`][pyhulax.video.stream.VideoStreamSimple]
- [`RTSPStream`][pyhulax.video.stream.RTSPStream]

Display helpers:

- [`VideoDisplay`][pyhulax.video.display.VideoDisplay]
- [`VideoDisplayAsync`][pyhulax.video.display.VideoDisplayAsync]
- [`show_frame`][pyhulax.video.display.show_frame]

Recording helpers:

- [`VideoRecorder`][pyhulax.video.recording.VideoRecorder]
- [`SegmentedRecorder`][pyhulax.video.recording.SegmentedRecorder]
- [`RecordingConfig`][pyhulax.video.recording.RecordingConfig]

Detection helpers:

- [`BaseDetector`][pyhulax.video.detection.BaseDetector]
- [`DetectorConfig`][pyhulax.video.detection.DetectorConfig]
- [`DrawDetections`][pyhulax.video.detection.DrawDetections]
- [`DetectionLogger`][pyhulax.video.detection.DetectionLogger]
- [`FilterDetector`][pyhulax.video.detection.FilterDetector]
- [`FrameCrop`][pyhulax.video.detection.FrameCrop]
- [`SaveDetectionCrop`][pyhulax.video.detection.SaveDetectionCrop]
- [`ONNXDetector`][pyhulax.video.detection.ONNXDetector]
- [`YOLODetector`][pyhulax.video.detection.YOLODetector]
- [`YOLOSegmentDetector`][pyhulax.video.detection.YOLOSegmentDetector]

Web helpers:

- [`MJPEGStreamer`][pyhulax.video.web.MJPEGStreamer]
- [`WebStreamServer`][pyhulax.video.web.WebStreamServer]

## Starting a Stream from [`DroneAPI`][pyhulax.DroneAPI]

```python
stream = drone.start_video_stream(display=True)
```

With browser streaming:

```python
stream = drone.start_video_stream(web_server=True)
```

`web_port=None` means the SDK uses [`config.network.web_port`][pyhulax.config.NetworkConfig.web_port].

Real-world pattern from the challenge scripts:

```python
from pyhulax import DroneAPI
from pyhulax.video import DrawDetections, ONNXDetector

with DroneAPI() as drone:
    drone.connect()
    detector = ONNXDetector(
        model_path="models/secrets.onnx",
        class_names=["0", "1", "2", "3", "4", "5"],
        confidence=0.5,
    )

    stream = drone.start_video_stream(display=False, web_server=True)
    stream.add_callback(detector)
    stream.add_callback(DrawDetections())

    try:
        stream.wait()
    finally:
        stream.stop()
```

## Creating a Stream Manually

```python
from pyhulax.video import VideoStream, VideoDisplay

stream = drone.create_video_stream()
stream.add_callback(VideoDisplay())

drone.set_video_stream(True)
stream.start()
```

This is useful when you need to control startup order yourself, for example enabling stream transport before attaching recorders or browser output.

## Frame Callback Pattern

Callbacks receive a [`Frame`][pyhulax.video.types.Frame] and return a [`Frame`][pyhulax.video.types.Frame] or `None`.

```python
def detect(frame):
    frame.metadata["source"] = "detector"
    return frame

stream.add_callback(detect)
```

The notebook and RTSP test script use the same callback pipeline idea for crop filters, detections, overlays, and recorders.

The important idea is that callbacks are composable. A stream can run a whole
pipeline in order:

1. mutate or filter the frame
2. run detection
3. draw overlays
4. record the result
5. display it or expose it to the web server

That is exactly how the challenge scripts and RTSP tests are structured.

```python
from pyhulax.video import Frame, RTSPStream, VideoDisplay, WebStreamServer

class CropFilter:
    def __call__(self, frame: Frame) -> Frame:
        frame.metadata["cropped"] = True
        return frame

stream = RTSPStream("rtsp://localhost:8554/stream", tcp_transport=False)
stream.add_callback(CropFilter())
stream.add_callback(VideoDisplay())
stream.start()

web = WebStreamServer(stream, port=8080)
web.start()
```

Because callbacks can mutate the frame in place, they are useful for:

- cropping or masking the image before inference
- attaching metadata for later callbacks
- drawing custom overlays
- saving side artifacts such as crops or debug images
- routing one stream to display, recording, and web output at once

Example of a lightweight metadata callback:

```python
from datetime import datetime
from pyhulax.video import Frame

def attach_debug_metadata(frame: Frame) -> Frame:
    frame.metadata["pipeline"] = "challenge2"
    frame.metadata["captured_at"] = datetime.now().isoformat()
    return frame

stream.add_callback(attach_debug_metadata)
```

Example of a filter callback that drops frames:

```python
from pyhulax.video import Frame

def keep_every_other_frame(frame: Frame) -> Frame | None:
    count = int(frame.metadata.get("count", 0)) + 1
    frame.metadata["count"] = count
    return frame if count % 2 == 0 else None

stream.add_callback(keep_every_other_frame)
```

Returning `None` is useful when you want to skip display, recording, or later
processing for selected frames.

Recording and detection can be layered into the same stream:

```python
from pyhulax.video import DrawDetections, RTSPStream, VideoRecorder, YOLODetector

stream = RTSPStream("rtsp://localhost:8554/stream", tcp_transport=True)
detector = YOLODetector(model_path="yolov8n.pt", confidence=0.5)
recorder = VideoRecorder("output.mp4", draw_detections=True)

stream.add_callback(detector)
stream.add_callback(DrawDetections())
stream.add_callback(recorder)
stream.start()
```

Typical challenge-style ordering:

```python
stream = drone.start_video_stream(display=False)
stream.add_callback(detector)
stream.add_callback(DrawDetections())
stream.add_callback(recorder)
```

That gives you:

- raw stream input from the drone
- detector output attached to frame metadata
- rendered boxes or labels on the frame
- saved video containing the overlays

## Logging Middleware

The SDK has two different lightweight file loggers and one structured flight
logger interface:

- [`FileLoggerMiddleware`][pyhulax.logging.FileLoggerMiddleware] logs incoming parsed MAVLink/state traffic
- [`CommandLogger`][pyhulax.logging.CommandLogger] logs outgoing `DroneAPI` method calls
- [`FlightLogger`][pyhulax.logging.FlightLogger] is the higher-level session/telemetry/command backend interface used by [`SQLiteLogger`][pyhulax.logging.SQLiteLogger] and [`PostgresLogger`][pyhulax.logging.PostgresLogger]

### `FileLoggerMiddleware`

[`FileLoggerMiddleware`][pyhulax.logging.FileLoggerMiddleware] writes newline-delimited JSON records like:

- timestamp
- MAVLink message type and ID
- serialized message fields
- optional parsed state payload

Files rotate daily as:

- `logs/drone_YYYY-MM-DD.jsonl`

Direct usage:

```python
from pyhulax.logging import FileLoggerMiddleware

logger = FileLoggerMiddleware("logs")

# typically called by the runtime when messages are parsed
logger.log_message(msg, state)
```

In normal `DroneAPI` usage this is enabled through:

```python
drone = DroneAPI(
    enable_file_logging=True,
    file_log_dir="logs",
)
```

This is useful when you want to inspect:

- raw telemetry flowing through the runtime
- message/state decoding behavior
- camera or QR response traffic
- failures that happen below the public API layer

### `CommandLogger`

[`CommandLogger`][pyhulax.logging.CommandLogger] writes JSONL records for public
API method calls. By default it skips high-frequency getters like
`get_position()` and `get_state()` to keep the logs readable.

Files rotate daily as:

- `logs/commands_YYYY-MM-DD.jsonl`

Direct usage:

```python
from pyhulax.logging import CommandLogger

logger = CommandLogger("logs")

@logger.log
def move(direction: str, distance: int) -> None:
    ...
```

Wrapping an object:

```python
from pyhulax import DroneAPI
from pyhulax.logging import CommandLogger, create_logging_wrapper

drone = DroneAPI(enable_command_logging=False)
logged_drone = create_logging_wrapper(drone, CommandLogger("logs"))

logged_drone.connect()
logged_drone.takeoff()
logged_drone.land()
```

The wrapper has special handling for `manual_fly()`: it injects an `on_frame`
callback so each individual `MANUAL_CONTROL` frame can be logged as well.

```python
logged_drone.manual_fly(2.0, forward=0.5, rotate=0.3)
```

That gives you both the top-level method call and the per-frame manual control
records.

## Logging

[`pyhulax.logging`][pyhulax.logging] exports:

- [`FlightLogger`][pyhulax.logging.FlightLogger]
- [`SQLiteLogger`][pyhulax.logging.SQLiteLogger]
- [`PostgresLogger`][pyhulax.logging.PostgresLogger]
- [`FileLoggerMiddleware`][pyhulax.logging.FileLoggerMiddleware]
- [`CommandLogger`][pyhulax.logging.CommandLogger]
- [`FlightSession`][pyhulax.logging.FlightSession]
- [`TelemetryRecord`][pyhulax.logging.TelemetryRecord]
- [`CommandRecord`][pyhulax.logging.CommandRecord]

## Flight Logging with [`DroneAPI`][pyhulax.DroneAPI]

```python
from pyhulax import DroneAPI
from pyhulax.logging import SQLiteLogger

logger = SQLiteLogger("flights.db")
drone = DroneAPI(flight_logger=logger)
```

Typical session:

```python
from pyhulax import DroneAPI
from pyhulax.logging import SQLiteLogger

logger = SQLiteLogger("flights.db")

with DroneAPI(flight_logger=logger) as drone:
    drone.connect()
    try:
        drone.takeoff()
        print(drone.get_state())
    finally:
        drone.land()
```

If you want database-backed telemetry history outside the `DroneAPI`
integration, the backend can also be used directly:

```python
from pyhulax.logging import SQLiteLogger

logger = SQLiteLogger("flights.db")
session_id = logger.start_session(drone_id=1, notes="Calibration run")
logger.log_telemetry(session_id, drone.get_flight_data())
logger.end_session(session_id)
```

## File Logging

`DroneAPI` also supports JSONL logging of parsed MAVLink traffic and API commands:

```python
drone = DroneAPI(
    enable_file_logging=True,
    file_log_dir="logs",
    enable_command_logging=True,
    command_log_dir="logs",
)
```

That produces local JSONL logs for command calls and parsed traffic, which is the easiest way to inspect real sessions during development.

This is separate from the database-backed [`FlightLogger`][pyhulax.logging.FlightLogger] interface.

## Notes

- [`SQLiteLogger`][pyhulax.logging.SQLiteLogger] is synchronous and suits local development.
- [`PostgresLogger`][pyhulax.logging.PostgresLogger] depends on the `db` extra.
- [`pyhulax.video`][pyhulax.video] raises an `ImportError` with an install hint if a backend is used without the required extra.
