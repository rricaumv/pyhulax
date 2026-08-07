"""Runner protocol + a hardware-free simulation runner.

A ``Runner`` drives one demo. Its ``run(opts, hooks)`` is called on a background
thread and pushes data to the UI through ``hooks``:

    hooks.should_stop() -> bool         # cooperative stop
    hooks.emit_frame(image, detections) # image: HxWx3 BGR ndarray;
                                         # detections: list of dicts
                                         # {x,y,w,h,label,conf} in image pixels
    hooks.emit_telemetry(dict)          # battery, height_cm, roll/pitch/yaw, ...
    hooks.emit_flight(dict)             # connection, phase, elapsed, detected, ...
    hooks.emit_log(str)

The UI supplies a ``hooks`` object (its RunnerThread) implementing this. Tests
supply a collecting stub. Nothing here imports PySide6 or pyhulax, so a real
flight runner added later can live alongside this and be tested the same way.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Protocol

import numpy as np


class Hooks(Protocol):
    def should_stop(self) -> bool: ...
    def emit_frame(self, image: "np.ndarray", detections: Optional[List[dict]] = None) -> None: ...
    def emit_telemetry(self, data: Dict[str, Any]) -> None: ...
    def emit_flight(self, data: Dict[str, Any]) -> None: ...
    def emit_log(self, message: str) -> None: ...


class Runner:
    """Base class. Implement ``run`` to drive a demo."""

    def run(self, opts: Dict[str, Any], hooks: Hooks) -> None:  # pragma: no cover
        raise NotImplementedError


class StubRunner(Runner):
    """Simulated runner: synthesizes video, detections, telemetry and a phase
    timeline so the whole UI can be exercised with no drone attached.

    Replace with a real ``Runner`` (one that drives ``pyhulax.DroneAPI`` and the
    demo's mission) to fly for real - the UI and hooks stay identical.
    """

    def __init__(self, phases: Optional[List[str]] = None,
                 width: int = 640, height: int = 480, phase_seconds: float = 2.5):
        self._phases = phases or ["idle"]
        self._w, self._h = width, height
        self._phase_seconds = phase_seconds

    # Phases in which a "tank" is considered visible in the simulation.
    _DETECT_PHASES = {"search", "center", "approach", "descend", "led", "flash"}

    def _blob_center(self, frame_no: int) -> "tuple[int, int]":
        cx = int(self._w * 0.5 + self._w * 0.22 * math.sin(frame_no * 0.05))
        cy = int(self._h * 0.58 + self._h * 0.10 * math.cos(frame_no * 0.05))
        return cx, cy

    def _make_frame(self, frame_no: int) -> "np.ndarray":
        h, w = self._h, self._w
        img = np.empty((h, w, 3), dtype=np.uint8)
        grad = np.linspace(30, 90, w, dtype=np.uint8)
        img[:, :, 0] = grad[None, :]          # B
        img[:, :, 1] = 55                      # G
        img[:, :, 2] = 40                      # R
        cx, cy = self._blob_center(frame_no)
        bw, bh = 60, 40
        x0, y0 = max(0, cx - bw // 2), max(0, cy - bh // 2)
        img[y0:cy + bh // 2, x0:cx + bw // 2] = (40, 70, 120)  # the "tank"
        return img

    def _detection(self, frame_no: int) -> List[dict]:
        cx, cy = self._blob_center(frame_no)
        return [{"x": cx - 30, "y": cy - 20, "w": 60, "h": 40,
                 "label": "tank", "conf": 0.87}]

    def run(self, opts: Dict[str, Any], hooks: Hooks) -> None:
        hooks.emit_log("SIMULATION: no drone connected (stub runner).")
        hooks.emit_log(f"ip={opts.get('ip', '-')} id={opts.get('id', '-')}  "
                       f"model={opts.get('model', '-')}")
        t0 = time.time()
        battery = 92.0
        target_h = float(opts.get("height", 100) or 100)
        frame_no = 0
        last_phase: Optional[str] = None
        total = self._phase_seconds * len(self._phases)

        while not hooks.should_stop():
            elapsed = time.time() - t0
            phase_idx = min(int(elapsed / self._phase_seconds), len(self._phases) - 1)
            phase = self._phases[phase_idx]
            if phase != last_phase:
                hooks.emit_log(f"phase -> {phase}")
                last_phase = phase

            detecting = phase in self._DETECT_PHASES
            dets = self._detection(frame_no) if detecting else None
            hooks.emit_frame(self._make_frame(frame_no), dets)

            battery = max(0.0, battery - 0.02)
            hooks.emit_telemetry({
                "battery": round(battery, 1),
                "height_cm": round(target_h + 5 * math.sin(elapsed), 1),
                "roll": round(3 * math.sin(elapsed * 1.3), 1),
                "pitch": round(3 * math.sin(elapsed * 0.9 + 1), 1),
                "yaw": round((elapsed * 20) % 360, 1),
                "vx": round(math.sin(elapsed), 2),
                "vy": round(math.cos(elapsed), 2),
                "vz": 0.0,
                "pos_x": round(10 * math.sin(elapsed * 0.3), 1),
                "pos_y": round(10 * math.cos(elapsed * 0.3), 1),
                "pos_z": round(target_h, 1),
            })
            hooks.emit_flight({
                "connection": "SIMULATED",
                "phase": phase,
                "elapsed": round(elapsed, 1),
                "detected": detecting,
                "fps": 20,
                "inference_ms": 12,
                "model": opts.get("model", "-"),
                "resolution": f"{self._w}x{self._h}",
                "frame": frame_no,
            })

            frame_no += 1
            if phase_idx == len(self._phases) - 1 and elapsed > total + 1.0:
                hooks.emit_log("SIMULATION complete.")
                break
            time.sleep(0.05)  # ~20 fps

        hooks.emit_log("runner stopped.")
