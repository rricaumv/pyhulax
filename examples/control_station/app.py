"""PySide6 control-station UI for the pyhulax demos.

Shell build: the layout, demo picker, auto-generated argument form, embedded
video (with detection overlays), telemetry / flight / info panels and log console
are all live, driven by the simulated ``StubRunner``. Wiring a demo for real
flight later means only swapping its ``runner_factory`` in ``registry.py`` for a
real ``Runner`` - this file does not change.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import numpy as np
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QPushButton, QScrollArea,
    QSpinBox, QSplitter, QVBoxLayout, QWidget,
)

from registry import DEMOS, ArgSpec, DemoSpec  # type: ignore
from runner import Runner  # type: ignore


# --------------------------------------------------------------------------- #
# Worker thread: runs a Runner and relays its hook calls as Qt signals.
# --------------------------------------------------------------------------- #
class RunnerThread(QThread):
    frameReady = Signal(object, object)   # image (ndarray), detections (list|None)
    telemetry = Signal(dict)
    flight = Signal(dict)
    logLine = Signal(str)

    def __init__(self, runner: Runner, opts: Dict[str, Any]):
        super().__init__()
        self._runner = runner
        self._opts = opts
        self._stop = False

    # --- Hooks protocol (called from this thread) ---
    def should_stop(self) -> bool:
        return self._stop

    def emit_frame(self, image, detections=None) -> None:
        self.frameReady.emit(image, detections)

    def emit_telemetry(self, data: Dict[str, Any]) -> None:
        self.telemetry.emit(data)

    def emit_flight(self, data: Dict[str, Any]) -> None:
        self.flight.emit(data)

    def emit_log(self, message: str) -> None:
        self.logLine.emit(message)

    # --- QThread ---
    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        try:
            self._runner.run(self._opts, self)
        except Exception as exc:  # noqa: BLE001
            self.logLine.emit(f"runner error: {exc}")


# --------------------------------------------------------------------------- #
# Argument form: builds widgets from a DemoSpec's ArgSpecs and reads them back.
# --------------------------------------------------------------------------- #
class ArgForm(QWidget):
    def __init__(self):
        super().__init__()
        self._outer = QVBoxLayout(self)
        self._outer.setContentsMargins(0, 0, 0, 0)
        self._widgets: Dict[str, tuple] = {}  # key -> (widget, kind)

    def build(self, spec: DemoSpec) -> None:
        # Clear previous form.
        while self._outer.count():
            item = self._outer.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._widgets.clear()

        # Group ArgSpecs by their `group` field, order of first appearance.
        groups: List[str] = []
        by_group: Dict[str, List[ArgSpec]] = {}
        for a in spec.args:
            by_group.setdefault(a.group, []).append(a)
            if a.group not in groups:
                groups.append(a.group)

        for group in groups:
            box = QGroupBox(group)
            form = QFormLayout(box)
            form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
            for a in by_group[group]:
                widget = self._make_widget(a)
                self._widgets[a.key] = (widget, a.kind)
                label = QLabel(a.flag)
                label.setToolTip(a.help)
                widget.setToolTip(a.help)
                form.addRow(label, widget)
            self._outer.addWidget(box)
        self._outer.addStretch(1)

    def _make_widget(self, a: ArgSpec) -> QWidget:
        if a.kind == "int":
            w = QSpinBox()
            w.setRange(int(a.minimum), int(a.maximum))
            w.setValue(int(a.default if a.default is not None else 0))
            return w
        if a.kind == "float":
            w = QDoubleSpinBox()
            w.setRange(float(a.minimum), float(a.maximum))
            w.setDecimals(3)
            w.setSingleStep(0.1)
            w.setValue(float(a.default if a.default is not None else 0.0))
            return w
        if a.kind == "bool":
            w = QCheckBox()
            w.setChecked(bool(a.default))
            return w
        if a.kind == "choice":
            w = QComboBox()
            w.addItems(a.choices or [])
            if a.default in (a.choices or []):
                w.setCurrentText(str(a.default))
            return w
        # str, int_list, str_list -> line edit
        w = QLineEdit()
        if a.default is not None:
            if isinstance(a.default, (list, tuple)):
                w.setText(" ".join(str(x) for x in a.default))
            else:
                w.setText(str(a.default))
        return w

    def collect(self) -> Dict[str, Any]:
        opts: Dict[str, Any] = {}
        for key, (w, kind) in self._widgets.items():
            if kind == "int":
                opts[key] = w.value()
            elif kind == "float":
                opts[key] = w.value()
            elif kind == "bool":
                opts[key] = w.isChecked()
            elif kind == "choice":
                opts[key] = w.currentText()
            elif kind == "int_list":
                opts[key] = [int(x) for x in w.text().split()] if w.text().strip() else []
            elif kind == "str_list":
                opts[key] = w.text().split()
            else:
                opts[key] = w.text()
        return opts

    def set_value(self, key: str, value: Any) -> None:
        """Override one field's value after build (e.g. per-drone id/ip)."""
        if key not in self._widgets:
            return
        w, kind = self._widgets[key]
        if kind == "int":
            w.setValue(int(value))
        elif kind == "float":
            w.setValue(float(value))
        elif kind == "bool":
            w.setChecked(bool(value))
        elif kind == "choice":
            w.setCurrentText(str(value))
        elif isinstance(value, (list, tuple)):
            w.setText(" ".join(str(x) for x in value))
        else:
            w.setText(str(value))


# Shared read-out definitions (used by the single- and dual-drone UIs).
TELEMETRY_FIELDS = [
    ("battery", "Battery %"), ("height_cm", "ToF height (cm)"),
    ("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw"),
    ("vx", "Vel X"), ("vy", "Vel Y"), ("vz", "Vel Z"),
    ("pos_x", "Pos X"), ("pos_y", "Pos Y"), ("pos_z", "Pos Z"),
]
TELEMETRY_FMT = {
    "battery": "{}%", "height_cm": "{} cm", "roll": "{}°", "pitch": "{}°",
    "yaw": "{}°", "vx": "{} cm/s", "vy": "{} cm/s", "vz": "{} cm/s",
    "pos_x": "{} cm", "pos_y": "{} cm", "pos_z": "{} cm",
}
FLIGHT_FIELDS = [
    ("connection", "Connection"), ("phase", "Phase"),
    ("elapsed", "Elapsed (s)"), ("detected", "Target seen"),
]
INFO_FIELDS = [
    ("model", "Model"), ("resolution", "Resolution"), ("fps", "FPS"),
    ("inference_ms", "Inference (ms)"), ("frame", "Frame #"),
]


# --------------------------------------------------------------------------- #
# Small labelled read-out panel (telemetry / flight / info).
# --------------------------------------------------------------------------- #
class ReadoutPanel(QGroupBox):
    def __init__(self, title: str, fields: List[tuple]):
        super().__init__(title)
        self._values: Dict[str, QLabel] = {}
        form = QFormLayout(self)
        mono = QFont("Monospace")
        mono.setStyleHint(QFont.StyleHint.TypeWriter)
        for key, label in fields:
            val = QLabel("-")
            val.setFont(mono)
            self._values[key] = val
            form.addRow(QLabel(label), val)

    def update_values(self, data: Dict[str, Any], fmt: Dict[str, str]) -> None:
        for key, label in self._values.items():
            if key in data:
                spec = fmt.get(key, "{}")
                try:
                    label.setText(spec.format(data[key]))
                except Exception:  # noqa: BLE001
                    label.setText(str(data[key]))


# --------------------------------------------------------------------------- #
# Video widget: shows frames + crosshair + detection boxes.
# --------------------------------------------------------------------------- #
class VideoView(QLabel):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(480, 360)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background:#111;color:#888;")
        self.setText("no video\n(press Start)")

    def show_frame(self, image: "np.ndarray", detections: Optional[List[dict]]) -> None:
        rgb = np.ascontiguousarray(image[:, :, ::-1])  # BGR -> RGB
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg)

        painter = QPainter(pix)
        # Crosshair at frame center.
        painter.setPen(QPen(QColor(0, 255, 255), 2))
        painter.drawLine(w // 2 - 12, h // 2, w // 2 + 12, h // 2)
        painter.drawLine(w // 2, h // 2 - 12, w // 2, h // 2 + 12)
        # Detection boxes.
        painter.setPen(QPen(QColor(0, 220, 0), 2))
        for d in detections or []:
            x, y, bw, bh = int(d["x"]), int(d["y"]), int(d["w"]), int(d["h"])
            painter.drawRect(x, y, bw, bh)
            painter.drawText(x, max(12, y - 4),
                             f"{d.get('label', '?')} {d.get('conf', 0):.2f}")
        painter.end()

        self.setPixmap(pix.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                  Qt.TransformationMode.SmoothTransformation))


# --------------------------------------------------------------------------- #
# Main window
# --------------------------------------------------------------------------- #
class ControlStation(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyhulax control station")
        self.resize(1200, 760)
        self._thread: Optional[RunnerThread] = None

        # --- Left: demo picker + argument form + controls ---
        self._demo_combo = QComboBox()
        for spec in DEMOS.values():
            self._demo_combo.addItem(spec.name, spec.key)
        self._demo_combo.currentIndexChanged.connect(self._on_demo_changed)

        self._desc = QLabel()
        self._desc.setWordWrap(True)
        self._desc.setStyleSheet("color:#aaa;")

        self._form = ArgForm()
        form_scroll = QScrollArea()
        form_scroll.setWidgetResizable(True)
        form_scroll.setWidget(self._form)
        form_scroll.setFrameShape(QFrame.Shape.NoFrame)

        self._start_btn = QPushButton("Start")
        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.clicked.connect(self._on_stop)
        self._stop_btn.setEnabled(False)
        btns = QHBoxLayout()
        btns.addWidget(self._start_btn)
        btns.addWidget(self._stop_btn)

        left = QVBoxLayout()
        left.addWidget(QLabel("Demo"))
        left.addWidget(self._demo_combo)
        left.addWidget(self._desc)
        left.addWidget(form_scroll, 1)
        left.addLayout(btns)
        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setMinimumWidth(320)
        left_w.setMaximumWidth(420)

        # --- Center: video ---
        self._video = VideoView()

        # --- Right: telemetry / flight / info ---
        self._telemetry = ReadoutPanel("Telemetry", TELEMETRY_FIELDS)
        self._tfmt = TELEMETRY_FMT
        self._flight = ReadoutPanel("Flight", FLIGHT_FIELDS)
        self._info = ReadoutPanel("Info", INFO_FIELDS)
        right = QVBoxLayout()
        right.addWidget(self._flight)
        right.addWidget(self._telemetry)
        right.addWidget(self._info)
        right.addStretch(1)
        right_w = QWidget()
        right_w.setLayout(right)
        right_w.setMinimumWidth(240)
        right_w.setMaximumWidth(320)

        top = QHBoxLayout()
        top.addWidget(left_w)
        top.addWidget(self._video, 1)
        top.addWidget(right_w)
        top_w = QWidget()
        top_w.setLayout(top)

        # --- Bottom: log ---
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)
        self._log.setStyleSheet("background:#0d0d0d;color:#ccc;font-family:monospace;")

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(top_w)
        splitter.addWidget(self._log)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)

        root = QVBoxLayout(self)
        root.addWidget(splitter)

        self._on_demo_changed()  # populate the form for the first demo

    # --- helpers ---
    def _current_spec(self) -> DemoSpec:
        return DEMOS[self._demo_combo.currentData()]

    def _on_demo_changed(self) -> None:
        spec = self._current_spec()
        self._desc.setText(spec.description)
        self._form.build(spec)

    def _log_line(self, msg: str) -> None:
        self._log.appendPlainText(msg)

    def _on_start(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            return
        spec = self._current_spec()
        opts = self._form.collect()
        self._log.clear()
        self._log_line(f"=== {spec.name} ===")
        runner = spec.runner_factory()
        self._thread = RunnerThread(runner, opts)
        self._thread.frameReady.connect(self._video.show_frame)
        self._thread.telemetry.connect(
            lambda d: self._telemetry.update_values(d, self._tfmt))
        self._thread.flight.connect(self._on_flight)
        self._thread.logLine.connect(self._log_line)
        self._thread.finished.connect(self._on_finished)
        self._demo_combo.setEnabled(False)
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._thread.start()

    def _on_flight(self, data: Dict[str, Any]) -> None:
        shown = dict(data)
        shown["detected"] = "yes" if data.get("detected") else "no"
        self._flight.update_values(shown, {})
        self._info.update_values(shown, {})

    def _on_stop(self) -> None:
        if self._thread is not None:
            self._thread.stop()
        self._stop_btn.setEnabled(False)

    def _on_finished(self) -> None:
        self._demo_combo.setEnabled(True)
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._log_line("--- finished ---")

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._thread is not None and self._thread.isRunning():
            self._thread.stop()
            self._thread.wait(2000)
        super().closeEvent(event)


def main() -> None:
    from PySide6.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)
    win = ControlStation()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
