"""Video helpers for the drone API.

Optional backends are lazy-loaded so importing the core API does not require
OpenCV, PyAV, ONNX Runtime, Flask, or YOLO dependencies.
"""

from __future__ import annotations

from importlib import import_module

from .types import (
    BoundingBox,
    Detection,
    Frame,
    FrameCallback,
    StreamConfig,
    StreamState,
)

_LAZY_EXPORTS = {
    "VideoStream": (".stream", "VideoStream", "Install the 'video' extra for streaming support."),
    "VideoStreamSimple": (".stream", "VideoStreamSimple", "Install the 'video' extra for streaming support."),
    "RTSPStream": (".stream", "RTSPStream", "Install the 'video' extra for streaming support."),
    "VideoDisplay": (".display", "VideoDisplay", "Install the 'video' extra for display support."),
    "VideoDisplayAsync": (".display", "VideoDisplayAsync", "Install the 'video' extra for display support."),
    "show_frame": (".display", "show_frame", "Install the 'video' extra for display support."),
    "VideoRecorder": (".recording", "VideoRecorder", "Install the 'video' extra for recording support."),
    "SegmentedRecorder": (".recording", "SegmentedRecorder", "Install the 'video' extra for recording support."),
    "RecordingConfig": (".recording", "RecordingConfig", "Install the 'video' extra for recording support."),
    "BaseDetector": (".detection", "BaseDetector", "Install the 'vision' extra for detection support."),
    "AsyncDetector": (".detection", "AsyncDetector", "Install the 'vision' extra for detection support."),
    "DetectorConfig": (".detection", "DetectorConfig", "Install the 'vision' extra for detection support."),
    "DrawDetections": (".detection", "DrawDetections", "Install the 'vision' extra for detection support."),
    "DetectionLogger": (".detection", "DetectionLogger", "Install the 'vision' extra for detection support."),
    "FilterDetector": (".detection", "FilterDetector", "Install the 'vision' extra for detection support."),
    "FrameCrop": (".detection", "FrameCrop", "Install the 'vision' extra for detection support."),
    "SaveDetectionCrop": (".detection", "SaveDetectionCrop", "Install the 'vision' extra for detection support."),
    "ONNXDetector": (".detection", "ONNXDetector", "Install the 'vision' extra for detection support."),
    "YOLODetector": (".detection", "YOLODetector", "Install the YOLO detection dependencies."),
    "YOLOSegmentDetector": (".detection", "YOLOSegmentDetector", "Install the YOLO detection dependencies."),
    "MJPEGStreamer": (".web", "MJPEGStreamer", "Install the 'web' extra for browser streaming support."),
    "WebStreamServer": (".web", "WebStreamServer", "Install the 'web' extra for browser streaming support."),
}


def __getattr__(name: str):
    if name not in _LAZY_EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attr_name, install_hint = _LAZY_EXPORTS[name]
    try:
        module = import_module(module_name, __name__)
    except ImportError as exc:  # pragma: no cover - depends on optional extras
        raise ImportError(f"{name} is unavailable. {install_hint}") from exc

    value = getattr(module, attr_name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(list(globals().keys()) + list(_LAZY_EXPORTS.keys()))

__all__ = [
    # Types
    "Frame",
    "Detection",
    "BoundingBox",
    "FrameCallback",
    "StreamState",
    "StreamConfig",
    # Stream classes
    "VideoStream",
    "VideoStreamSimple",
    "RTSPStream",
    # Display classes
    "VideoDisplay",
    "VideoDisplayAsync",
    "show_frame",
    # Recording
    "VideoRecorder",
    "SegmentedRecorder",
    "RecordingConfig",
    # Detection
    "BaseDetector",
    "AsyncDetector",
    "DetectorConfig",
    "ONNXDetector",
    "YOLODetector",
    "YOLOSegmentDetector",
    "FilterDetector",
    "DrawDetections",
    "DetectionLogger",
    "FrameCrop",
    "SaveDetectionCrop",
    # Web streaming (optional)
    "MJPEGStreamer",
    "WebStreamServer",
]
