"""
Object detection integration for video streaming.

Provides YOLO and other detector wrappers as VideoStream callbacks.
"""

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Union, Any
from pathlib import Path

import numpy as np

from .types import Frame, Detection, BoundingBox


@dataclass
class DetectorConfig:
    """Configuration for object detectors."""
    confidence_threshold: float = 0.25
    iou_threshold: float = 0.45  # NMS IoU threshold
    max_detections: int = 100
    classes: Optional[List[int]] = None  # Filter to specific class IDs
    device: str = "auto"  # "auto", "cpu", "cuda", "cuda:0", "mps"
    # Inference precision (ultralytics `quantize`, replaces the deprecated
    # `half`): 16/"fp16" for FP16, 32/"fp32"/None for FP32, 8 for int8.
    quantize: Optional[Union[int, str]] = None
    verbose: bool = False


class BaseDetector(ABC):
    """
    Abstract base class for object detectors.

    Subclass this to integrate custom detection models.
    """

    def __init__(self, config: Optional[DetectorConfig] = None):
        self.config = config or DetectorConfig()
        self._inference_times: List[float] = []
        self._max_time_samples = 100

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run detection on an image.

        Args:
            image: BGR image (numpy array)

        Returns:
            List of Detection objects
        """
        pass

    @property
    def avg_inference_time(self) -> float:
        """Average inference time in milliseconds."""
        if not self._inference_times:
            return 0.0
        return sum(self._inference_times) / len(self._inference_times) * 1000

    def _record_time(self, elapsed: float) -> None:
        """Record inference time for statistics."""
        self._inference_times.append(elapsed)
        if len(self._inference_times) > self._max_time_samples:
            self._inference_times.pop(0)

    def __call__(self, frame: Frame) -> Frame:
        """
        Callback interface for VideoStream.

        Args:
            frame: Input frame

        Returns:
            Frame with detections added
        """
        start = time.perf_counter()
        frame.detections = self.detect(frame.image)
        self._record_time(time.perf_counter() - start)
        return frame


class YOLODetector(BaseDetector):
    """
    YOLO object detector using ultralytics library.

    Supports YOLOv8, YOLOv9, YOLOv10, YOLO11, and YOLO-World models.

    Example:
    ```python
    from pyhulax.video import VideoStream, YOLODetector, VideoDisplay

    # Basic usage
    detector = YOLODetector("yolov8n.pt")
    stream = VideoStream(drone_ip="192.168.100.1")
    stream.add_callback(detector)
    stream.add_callback(VideoDisplay())
    stream.start()

    # With custom configuration
    detector = YOLODetector(
        model_path="yolov8s.pt",
        confidence=0.5,
        classes=[0, 1, 2],  # person, bicycle, car
        device="cuda",
    )

    # Using YOLO-World for open vocabulary detection
    detector = YOLODetector("yolov8s-world.pt")
    detector.set_classes(["person", "drone", "car"])
    ```
    """

    def __init__(
        self,
        model_path: Union[str, Path] = "yolov8n.pt",
        confidence: float = 0.25,
        iou_threshold: float = 0.45,
        classes: Optional[List[int]] = None,
        device: str = "auto",
        quantize: Optional[Union[int, str]] = None,
        verbose: bool = False,
        imgsz: int = 640,
        half: Optional[bool] = None,
    ):
        """
        Initialize YOLO detector.

        Args:
            model_path: Path to YOLO model (.pt file) or model name
                       (e.g., "yolov8n.pt", "yolov8s.pt", "yolov8m.pt")
            confidence: Minimum confidence threshold (0-1)
            iou_threshold: NMS IoU threshold
            classes: List of class IDs to detect (None for all)
            device: Device to run on ("auto", "cpu", "cuda", "cuda:0", "mps")
            quantize: Inference precision (ultralytics arg, replaces `half`):
                     16/"fp16" for FP16, 32/"fp32"/None for FP32, 8 for int8.
            verbose: Print detection info
            imgsz: Input image size for inference
            half: Deprecated. Use quantize=16 for FP16 instead.
        """
        if half is not None:
            import warnings

            warnings.warn(
                "'half' is deprecated; use quantize=16 (FP16) instead.",
                DeprecationWarning,
                stacklevel=2,
            )
            if quantize is None:
                quantize = 16 if half else None

        config = DetectorConfig(
            confidence_threshold=confidence,
            iou_threshold=iou_threshold,
            classes=classes,
            device=device,
            quantize=quantize,
            verbose=verbose,
        )
        super().__init__(config)

        self._model_path = str(model_path)
        self._imgsz = imgsz
        self._model = None
        self._class_names: dict = {}

    def _load_model(self) -> None:
        """Lazy load the YOLO model."""
        if self._model is not None:
            return

        try:
            from ultralytics import YOLO
        except ImportError:
            raise ImportError(
                "ultralytics not installed. Install with: pip install ultralytics"
            )

        self._model = YOLO(self._model_path)

        # Set device
        if self.config.device != "auto":
            self._model.to(self.config.device)

        # Get class names
        self._class_names = self._model.names

    @property
    def class_names(self) -> dict:
        """Get model class names (loads model if needed)."""
        self._load_model()
        return self._class_names

    def set_classes(self, classes: List[str]) -> None:
        """
        Set classes for YOLO-World open vocabulary detection.

        Args:
            classes: List of class names to detect
        """
        self._load_model()
        if hasattr(self._model, 'set_classes'):
            self._model.set_classes(classes)
        else:
            raise RuntimeError(
                "set_classes() only works with YOLO-World models. "
                "Use a model like 'yolov8s-world.pt'"
            )

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run YOLO detection on image.

        Args:
            image: BGR image (numpy array)

        Returns:
            List of Detection objects
        """
        self._load_model()

        # Run inference
        results = self._model(
            image,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            classes=self.config.classes,
            verbose=self.config.verbose,
            quantize=self.config.quantize,
            imgsz=self._imgsz,
        )

        detections = []
        if len(results) > 0:
            result = results[0]

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self._class_names.get(cls_id, str(cls_id))

                detections.append(
                    Detection(
                        label=label,
                        confidence=conf,
                        class_id=cls_id,
                        bbox=BoundingBox(
                            x=int(x1),
                            y=int(y1),
                            width=int(x2 - x1),
                            height=int(y2 - y1),
                        ),
                    )
                )

        return detections


class YOLOSegmentDetector(YOLODetector):
    """
    YOLO instance segmentation detector.

    Returns detections with segmentation masks in metadata.

    Example:
    ```python
    detector = YOLOSegmentDetector("yolov8n-seg.pt")
    stream.add_callback(detector)

    # Access masks
    for det in frame.detections:
        mask = det.metadata.get("mask")  # Binary mask
    ```
    """

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run YOLO segmentation."""
        self._load_model()

        results = self._model(
            image,
            conf=self.config.confidence_threshold,
            iou=self.config.iou_threshold,
            classes=self.config.classes,
            verbose=self.config.verbose,
            quantize=self.config.quantize,
            imgsz=self._imgsz,
        )

        detections = []
        if len(results) > 0 and results[0].masks is not None:
            result = results[0]
            masks = result.masks.data.cpu().numpy()

            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self._class_names.get(cls_id, str(cls_id))

                detections.append(
                    Detection(
                        label=label,
                        confidence=conf,
                        class_id=cls_id,
                        bbox=BoundingBox(
                            x=int(x1),
                            y=int(y1),
                            width=int(x2 - x1),
                            height=int(y2 - y1),
                        ),
                        metadata={"mask": masks[i]},
                    )
                )

        return detections


class FilterDetector(BaseDetector):
    """
    Wrapper that filters detections from another detector.

    Useful for filtering by class, confidence, size, or region.

    Example:
    ```python
    # Only detect persons with high confidence
    detector = YOLODetector("yolov8n.pt")
    filtered = FilterDetector(
        detector,
        classes=["person"],
        min_confidence=0.7,
    )
    stream.add_callback(filtered)
    ```
    """

    def __init__(
        self,
        detector: BaseDetector,
        classes: Optional[List[str]] = None,
        min_confidence: float = 0.0,
        min_area: int = 0,
        max_area: Optional[int] = None,
        roi: Optional[Tuple[int, int, int, int]] = None,
    ):
        """
        Initialize filter.

        Args:
            detector: Underlying detector
            classes: List of class names to keep (None for all)
            min_confidence: Minimum confidence threshold
            min_area: Minimum bounding box area in pixels
            max_area: Maximum bounding box area (None for no limit)
            roi: Region of interest (x, y, width, height) - only keep
                 detections with centers inside this region
        """
        super().__init__(detector.config)
        self._detector = detector
        self._classes = set(classes) if classes else None
        self._min_confidence = min_confidence
        self._min_area = min_area
        self._max_area = max_area
        self._roi = roi

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run detection with filtering."""
        detections = self._detector.detect(image)

        filtered = []
        for det in detections:
            # Class filter
            if self._classes and det.label not in self._classes:
                continue

            # Confidence filter
            if det.confidence < self._min_confidence:
                continue

            # Area filter
            area = det.bbox.area
            if area < self._min_area:
                continue
            if self._max_area and area > self._max_area:
                continue

            # ROI filter
            if self._roi:
                cx, cy = det.bbox.center
                rx, ry, rw, rh = self._roi
                if not (rx <= cx <= rx + rw and ry <= cy <= ry + rh):
                    continue

            filtered.append(det)

        return filtered


class DrawDetections:
    """
    Callback that draws detections on frames.

    Use after a detector in the pipeline to visualize results.

    Example:
    ```python
    stream.add_callback(detector)
    stream.add_callback(DrawDetections())
    stream.add_callback(display)
    ```
    """

    def __init__(
        self,
        thickness: int = 2,
        font_scale: float = 0.6,
        show_confidence: bool = True,
        show_label: bool = True,
    ):
        self._thickness = thickness
        self._font_scale = font_scale
        self._show_confidence = show_confidence
        self._show_label = show_label

    def __call__(self, frame: Frame) -> Frame:
        """Draw detections on frame."""
        if frame.detections:
            frame.image = frame.draw_detections(
                thickness=self._thickness,
                font_scale=self._font_scale,
            )
        return frame


class DetectionLogger:
    """
    Callback that logs detections for analysis.

    Example:
    ```python
    logger = DetectionLogger()
    stream.add_callback(detector)
    stream.add_callback(logger)
    # ... stream ...
    print(logger.get_summary())
    ```
    """

    def __init__(self):
        self._detections: List[Tuple[int, float, List[Detection]]] = []
        self._class_counts: dict = {}

    def __call__(self, frame: Frame) -> Frame:
        """Log detections from frame."""
        if frame.detections:
            self._detections.append(
                (frame.frame_number, frame.timestamp, frame.detections)
            )
            for det in frame.detections:
                self._class_counts[det.label] = self._class_counts.get(det.label, 0) + 1
        return frame

    @property
    def total_detections(self) -> int:
        """Total number of detections logged."""
        return sum(len(dets) for _, _, dets in self._detections)

    @property
    def class_counts(self) -> dict:
        """Detection counts per class."""
        return dict(self._class_counts)

    def get_summary(self) -> str:
        """Get detection summary string."""
        lines = [
            f"Total frames with detections: {len(self._detections)}",
            f"Total detections: {self.total_detections}",
            "Counts by class:",
        ]
        for cls, count in sorted(self._class_counts.items(), key=lambda x: -x[1]):
            lines.append(f"  {cls}: {count}")
        return "\n".join(lines)

    def clear(self) -> None:
        """Clear logged data."""
        self._detections.clear()
        self._class_counts.clear()


class FrameCrop:
    """
    Callback that crops margins from video frames.

    Use BEFORE detection to remove unwanted regions (e.g., propellers, sky, ground).
    Detection coordinates will be relative to the cropped frame.

    Example:
    ```python
    # Remove top 300px and bottom 200px from each frame
    crop = FrameCrop(top=300, bottom=200)
    stream.add_callback(crop)       # 1. Crop first
    stream.add_callback(detector)   # 2. Then detect
    stream.add_callback(display)
    ```
    """

    def __init__(
        self,
        top: int = 0,
        bottom: int = 0,
        left: int = 0,
        right: int = 0,
    ):
        """
        Initialize frame cropper.

        Args:
            top: Pixels to remove from top
            bottom: Pixels to remove from bottom
            left: Pixels to remove from left
            right: Pixels to remove from right
        """
        self.top = max(0, top)
        self.bottom = max(0, bottom)
        self.left = max(0, left)
        self.right = max(0, right)
        self.enabled = True

    def __call__(self, frame: Frame) -> Frame:
        """Crop margins from frame."""
        if not self.enabled:
            return frame

        h, w = frame.image.shape[:2]

        # Validate crop doesn't exceed image size
        if self.top + self.bottom >= h or self.left + self.right >= w:
            return frame

        y1 = self.top
        y2 = h - self.bottom
        x1 = self.left
        x2 = w - self.right

        frame.image = frame.image[y1:y2, x1:x2]
        return frame


class SaveDetectionCrop:
    """
    Callback that saves cropped images of detected objects.

    Use AFTER detection to save each detected object as a separate image file.

    Example:
    ```python
    saver = SaveDetectionCrop(save_dir="detections", one_per_class=True)
    stream.add_callback(detector)   # 1. Detect first
    stream.add_callback(saver)      # 2. Save crops
    stream.add_callback(display)

    # After streaming, check saved images:
    print(saver.saved_files)  # List of saved file paths
    ```
    """

    def __init__(
        self,
        save_dir: str = "detections",
        one_per_class: bool = True,
        min_confidence: float = 0.0,
        filename_format: str = "{label}_{confidence:.2f}_{frame}.jpg",
    ):
        """
        Initialize detection crop saver.

        Args:
            save_dir: Directory to save cropped images
            one_per_class: If True, only save first detection per class label
            min_confidence: Minimum confidence threshold to save
            filename_format: Format string for filenames. Available variables:
                            {label}, {confidence}, {frame}, {timestamp}
        """
        import os
        self._save_dir = save_dir
        self._one_per_class = one_per_class
        self._min_confidence = min_confidence
        self._filename_format = filename_format
        self._saved_classes: set = set()
        self._saved_files: List[str] = []
        os.makedirs(save_dir, exist_ok=True)

    @property
    def saved_files(self) -> List[str]:
        """List of saved file paths."""
        return list(self._saved_files)

    @property
    def saved_classes(self) -> set:
        """Set of class labels that have been saved."""
        return set(self._saved_classes)

    def reset(self) -> None:
        """Reset saved state to allow saving again."""
        self._saved_classes.clear()
        self._saved_files.clear()

    def __call__(self, frame: Frame) -> Frame:
        """Save cropped images of detections."""
        import os
        import cv2

        for det in frame.detections:
            # Skip low confidence
            if det.confidence < self._min_confidence:
                continue

            # Skip if already saved this class
            if self._one_per_class and det.label in self._saved_classes:
                continue

            # Get bounding box coordinates
            x1, y1, x2, y2 = det.bbox.to_xyxy()

            # Clamp to image bounds
            h, w = frame.image.shape[:2]
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w, x2)
            y2 = min(h, y2)

            # Skip invalid boxes
            if x2 <= x1 or y2 <= y1:
                continue

            # Crop the detection
            cropped = frame.image[y1:y2, x1:x2]

            # Generate filename
            filename = self._filename_format.format(
                label=det.label,
                confidence=det.confidence,
                frame=frame.frame_number,
                timestamp=int(frame.timestamp),
            )
            filepath = os.path.join(self._save_dir, filename)

            # Save image
            cv2.imwrite(filepath, cropped)
            self._saved_files.append(filepath)

            if self._one_per_class:
                self._saved_classes.add(det.label)

        return frame


class ONNXDetector(BaseDetector):
    """
    ONNX Runtime detector for YOLO-style models.

    Handles models with output shape [1, 4+num_classes, num_boxes].
    Applies NMS to filter detections.

    Example:
    ```python
    from pyhulax.video import ONNXDetector, DrawDetections, VideoDisplay

    detector = ONNXDetector(
        model_path="model.onnx",
        class_names=["House", "Tank", "Tree"],
        confidence=0.3,
    )

    stream = drone.start_video_stream(display=False)
    stream.add_callback(detector)
    stream.add_callback(DrawDetections())
    stream.add_callback(VideoDisplay())
    ```
    """

    def __init__(
        self,
        model_path: Union[str, Path],
        class_names: List[str],
        confidence: float = 0.25,
        iou_threshold: float = 0.45,
        input_size: Tuple[int, int] = (640, 640),
    ):
        """
        Initialize ONNX detector.

        Args:
            model_path: Path to ONNX model file
            class_names: List of class names in order of model output
            confidence: Minimum confidence threshold (0-1)
            iou_threshold: NMS IoU threshold
            input_size: Model input size (width, height)
        """
        config = DetectorConfig(
            confidence_threshold=confidence,
            iou_threshold=iou_threshold,
        )
        super().__init__(config)

        self._model_path = str(model_path)
        self._class_names = class_names
        self._input_size = input_size
        self._session = None

    def _load_model(self) -> None:
        """Lazy load the ONNX model."""
        if self._session is not None:
            return

        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError(
                "onnxruntime not installed. Install with: pip install onnxruntime"
            )

        self._session = ort.InferenceSession(
            self._model_path,
            providers=["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name

    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        Preprocess BGR image for inference.

        Args:
            image: BGR image (numpy array)

        Returns:
            Tuple of (input tensor, original size)
        """
        import cv2

        orig_h, orig_w = image.shape[:2]

        # Resize to model input size
        resized = cv2.resize(image, self._input_size)

        # BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # HWC to NCHW, normalize to 0-1
        tensor = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        tensor = np.expand_dims(tensor, axis=0)

        return tensor, (orig_w, orig_h)

    def _postprocess(
        self,
        output: np.ndarray,
        orig_size: Tuple[int, int],
    ) -> List[Detection]:
        """
        Parse YOLO-style output and apply NMS.

        Args:
            output: Model output with shape [1, 4+num_classes, num_boxes]
            orig_size: Original image size (width, height)

        Returns:
            List of Detection objects
        """
        import cv2

        # Transpose to [num_boxes, 4+num_classes]
        predictions = output[0].T

        orig_w, orig_h = orig_size
        input_w, input_h = self._input_size
        scale_x = orig_w / input_w
        scale_y = orig_h / input_h

        # Extract boxes and class scores
        boxes_xywh = predictions[:, :4]  # x_center, y_center, w, h
        class_scores = predictions[:, 4:]  # class confidences

        # Get best class per box
        class_ids = np.argmax(class_scores, axis=1)
        confidences = np.max(class_scores, axis=1)

        # Filter by confidence
        mask = confidences >= self.config.confidence_threshold
        boxes_xywh = boxes_xywh[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]

        if len(boxes_xywh) == 0:
            return []

        # Convert xywh to xyxy
        boxes_xyxy = np.zeros_like(boxes_xywh)
        boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2  # y2

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(
            boxes_xyxy.tolist(),
            confidences.tolist(),
            self.config.confidence_threshold,
            self.config.iou_threshold,
        )

        detections = []
        for i in indices:
            idx = i[0] if isinstance(i, (list, np.ndarray)) else i
            x1, y1, x2, y2 = boxes_xyxy[idx]

            # Scale to original image size
            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)

            cls_id = int(class_ids[idx])
            label = self._class_names[cls_id] if cls_id < len(self._class_names) else str(cls_id)

            detections.append(
                Detection(
                    label=label,
                    confidence=float(confidences[idx]),
                    class_id=cls_id,
                    bbox=BoundingBox(x=x1, y=y1, width=x2 - x1, height=y2 - y1),
                )
            )

        return detections

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run detection on image.

        Args:
            image: BGR image (numpy array)

        Returns:
            List of Detection objects
        """
        self._load_model()
        tensor, orig_size = self._preprocess(image)
        outputs = self._session.run(None, {self._input_name: tensor})
        return self._postprocess(outputs[0], orig_size)
