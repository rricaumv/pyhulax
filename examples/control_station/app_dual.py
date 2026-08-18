"""Two-drone control station.

A separate version of the control station that commands two drones at once: two
independent drone panels (each with its own demo picker, argument form, embedded
video + detections, telemetry / flight / info read-outs, and Start/Stop), a
"Start both / Stop both" bar, and a shared log. Each panel drives its own
``RunnerThread`` with its own ``opts`` (distinct id / ip), so the two run
concurrently - matching the SDK's per-drone identity model.

Reuses the single-drone widgets (``ArgForm``, ``ReadoutPanel``, ``VideoView``,
``RunnerThread``) and the demo registry, so registering a demo once makes it
available in both the single- and dual-drone UIs.

Run:  python examples/control_station/app_dual.py
  or  python examples/control_station/__main__.py --dual
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox, QFrame, QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit,
    QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget,
)

from app import (  # type: ignore  # sibling module (reused leaf widgets/consts)
    ArgForm, FLIGHT_FIELDS, INFO_FIELDS, ReadoutPanel, RunnerThread,
    TELEMETRY_FIELDS, TELEMETRY_FMT, VideoView,
)
from registry import DEMOS  # type: ignore


class DronePanel(QGroupBox):
    """One drone's controls + video + read-outs, self-contained."""

    logLine = Signal(str)  # prefixed lines forwarded to the shared log

    def __init__(self, title: str, id_default: Optional[int] = None,
                 ip_default: Optional[str] = None):
        super().__init__(title)
        self._title = title
        self._id_default = id_default
        self._ip_default = ip_default
        self._thread: Optional[RunnerThread] = None

        # Controls column.
        self._combo = QComboBox()
        for spec in DEMOS.values():
            self._combo.addItem(spec.name, spec.key)
        self._combo.currentIndexChanged.connect(self._on_demo_changed)
        self._desc = QLabel()
        self._desc.setWordWrap(True)
        self._desc.setStyleSheet("color:#aaa;")
        self._form = ArgForm()
        form_scroll = QScrollArea()
        form_scroll.setWidgetResizable(True)
        form_scroll.setWidget(self._form)
        form_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._start = QPushButton("Start")
        self._start.clicked.connect(self.start)
        self._stop = QPushButton("Stop")
        self._stop.clicked.connect(self.stop)
        self._stop.setEnabled(False)
        btns = QHBoxLayout()
        btns.addWidget(self._start)
        btns.addWidget(self._stop)
        controls = QVBoxLayout()
        controls.addWidget(self._combo)
        controls.addWidget(self._desc)
        controls.addWidget(form_scroll, 1)
        controls.addLayout(btns)
        controls_w = QWidget()
        controls_w.setLayout(controls)
        controls_w.setMinimumWidth(280)
        controls_w.setMaximumWidth(340)

        # Video.
        self._video = VideoView()
        self._video.setMinimumSize(320, 240)

        # Read-outs column.
        self._telemetry = ReadoutPanel("Telemetry", TELEMETRY_FIELDS)
        self._flight = ReadoutPanel("Flight", FLIGHT_FIELDS)
        self._info = ReadoutPanel("Info", INFO_FIELDS)
        readouts = QVBoxLayout()
        readouts.addWidget(self._flight)
        readouts.addWidget(self._telemetry)
        readouts.addWidget(self._info)
        readouts.addStretch(1)
        readouts_w = QWidget()
        readouts_w.setLayout(readouts)
        readouts_w.setMinimumWidth(220)
        readouts_w.setMaximumWidth(300)

        row = QHBoxLayout(self)
        row.addWidget(controls_w)
        row.addWidget(self._video, 1)
        row.addWidget(readouts_w)

        self._on_demo_changed()

    # --- helpers ---
    def _current_spec(self):
        return DEMOS[self._combo.currentData()]

    def _on_demo_changed(self) -> None:
        spec = self._current_spec()
        self._desc.setText(spec.description)
        self._form.build(spec)
        if self._id_default is not None:
            self._form.set_value("id", self._id_default)
        if self._ip_default:
            self._form.set_value("ip", self._ip_default)

    def _emit(self, msg: str) -> None:
        self.logLine.emit(f"[{self._title}] {msg}")

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.isRunning()

    def start(self) -> None:
        if self.is_running():
            return
        spec = self._current_spec()
        opts = self._form.collect()
        self._emit(f"=== {spec.name} (id={opts.get('id')}, ip={opts.get('ip')}) ===")
        self._thread = RunnerThread(spec.runner_factory(), opts)
        self._thread.frameReady.connect(self._video.show_frame)
        self._thread.telemetry.connect(
            lambda d: self._telemetry.update_values(d, TELEMETRY_FMT))
        self._thread.flight.connect(self._on_flight)
        self._thread.logLine.connect(self._emit)
        self._thread.finished.connect(self._on_finished)
        self._combo.setEnabled(False)
        self._start.setEnabled(False)
        self._stop.setEnabled(True)
        self._thread.start()

    def _on_flight(self, data: Dict[str, Any]) -> None:
        shown = dict(data)
        shown["detected"] = "yes" if data.get("detected") else "no"
        self._flight.update_values(shown, {})
        self._info.update_values(shown, {})

    def stop(self) -> None:
        if self._thread is not None:
            self._thread.stop()
        self._stop.setEnabled(False)

    def _on_finished(self) -> None:
        self._combo.setEnabled(True)
        self._start.setEnabled(True)
        self._stop.setEnabled(False)
        self._emit("finished")

    def shutdown(self) -> None:
        if self.is_running():
            self._thread.stop()
            self._thread.wait(2000)


class DualControlStation(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("pyhulax control station - two drones")
        self.resize(1300, 900)

        self._panel_a = DronePanel("Drone A", id_default=1, ip_default="192.168.1.49")
        self._panel_b = DronePanel("Drone B", id_default=2, ip_default="192.168.1.50")
        self._panels = [self._panel_a, self._panel_b]

        start_both = QPushButton("Start both")
        start_both.clicked.connect(self._start_both)
        stop_both = QPushButton("Stop both")
        stop_both.clicked.connect(self._stop_both)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("<b>Two-drone control station</b>"))
        bar.addStretch(1)
        bar.addWidget(start_both)
        bar.addWidget(stop_both)
        bar_w = QWidget()
        bar_w.setLayout(bar)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(800)
        self._log.setStyleSheet("background:#0d0d0d;color:#ccc;font-family:monospace;")
        for panel in self._panels:
            panel.logLine.connect(self._log.appendPlainText)

        panels = QSplitter(Qt.Orientation.Vertical)
        panels.addWidget(self._panel_a)
        panels.addWidget(self._panel_b)
        panels.setStretchFactor(0, 1)
        panels.setStretchFactor(1, 1)

        outer = QSplitter(Qt.Orientation.Vertical)
        outer.addWidget(panels)
        outer.addWidget(self._log)
        outer.setStretchFactor(0, 5)
        outer.setStretchFactor(1, 1)

        root = QVBoxLayout(self)
        root.addWidget(bar_w)
        root.addWidget(outer, 1)

    def _start_both(self) -> None:
        for panel in self._panels:
            panel.start()

    def _stop_both(self) -> None:
        for panel in self._panels:
            panel.stop()

    def closeEvent(self, event) -> None:  # noqa: N802
        for panel in self._panels:
            panel.shutdown()
        super().closeEvent(event)


def main() -> None:
    from PySide6.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)
    win = DualControlStation()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
