"""Real (flying) runners for the control station.

These import pyhulax and the demo modules lazily inside ``run`` so the registry,
the simulated runner, and the headless tests stay free of heavy/optional
dependencies. A real runner sets up the drone + video + detector, then drives the
demo's existing mission function, forwarding frames, detections, telemetry, the
mission phase, and logs to the UI through the same ``hooks`` the simulation uses.
"""

from __future__ import annotations

import importlib.util
import os
import threading
import time
from typing import Any, Dict

from runner import Runner  # type: ignore  # sibling module

_DEMO_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "mini_tank_approach_demo.py",
)
_demo_module = None


def _load_demo():
    """Import examples/mini_tank_approach_demo.py once and cache it."""
    global _demo_module
    if _demo_module is None:
        spec = importlib.util.spec_from_file_location("mini_tank_approach_demo", _DEMO_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _demo_module = mod
    return _demo_module


def _opts_to_argv(gui_opts: Dict[str, Any]) -> list:
    """Turn the GUI's opts subset into CLI argv for the demo's build_opts().

    Keys map to flags by ``--{key-with-dashes}``, matching the demo's argparse.
    """
    argv = []
    for key, value in gui_opts.items():
        argv += [f"--{key.replace('_', '-')}", str(value)]
    return argv


class MiniTankRunner(Runner):
    """Flies examples/mini_tank_approach_demo.py inside the control station."""

    def run(self, gui_opts: Dict[str, Any], hooks) -> None:
        try:
            demo = _load_demo()
            from pyhulax import DroneAPI
            from pyhulax.video import AsyncDetector, YOLODetector
        except Exception as exc:  # noqa: BLE001
            hooks.emit_log(f"cannot start (missing deps?): {exc}")
            hooks.emit_log("  install:  pip install 'pyhulax[video]' ultralytics")
            return

        # Full opts = demo defaults overlaid with the GUI form values.
        opts, _, _ = demo.build_opts(_opts_to_argv(gui_opts))

        state: Dict[str, Any] = {"phase": "init"}
        frames: Dict[str, Any] = {}
        lock = threading.Lock()
        stop_event = threading.Event()
        key = f"D{opts['id']}"
        t0 = time.time()

        # Bridge the GUI Stop button -> the mission's stop_event.
        def _watch_stop():
            while not stop_event.is_set():
                if hooks.should_stop():
                    stop_event.set()
                    return
                time.sleep(0.1)

        threading.Thread(target=_watch_stop, daemon=True).start()

        drone = DroneAPI(drone_id=opts["id"])
        hooks.emit_flight({"connection": "connecting", "phase": "init"})
        try:
            hooks.emit_log(f"connecting to {opts['ip']} ...")
            drone.connect(opts["ip"], timeout=opts["connect_timeout"])
        except Exception as exc:  # noqa: BLE001
            hooks.emit_log(f"CONNECT FAILED: {exc}")
            hooks.emit_flight({"connection": "failed", "phase": "init"})
            stop_event.set()
            return

        try:
            detector = YOLODetector(model_path=opts["model"], confidence=opts["confidence"],
                                    classes=opts["classes"], imgsz=opts["imgsz"])
        except Exception as exc:  # noqa: BLE001
            hooks.emit_log(f"detector unavailable: {exc}")
            drone.disconnect()
            stop_event.set()
            return
        adet = AsyncDetector(detector)

        drone.set_video_stream(True)
        stream = drone.create_video_stream()

        def _capture(frame):
            with lock:
                frames[key] = frame
            return frame

        def _emit(frame):
            # Push frame + detections to the UI (box kept on in every phase); the
            # UI draws the box from these dicts plus the crosshair.
            dets = [{"x": d.bbox.x, "y": d.bbox.y, "w": d.bbox.width,
                     "h": d.bbox.height, "label": d.label, "conf": d.confidence}
                    for d in (frame.detections or [])]
            hooks.emit_frame(frame.image, dets)
            return frame

        stream.add_callback(adet)      # off-thread detection
        stream.add_callback(_capture)  # stash for the mission's frame_size/current_frame
        stream.add_callback(_emit)     # push to the UI
        stream.start()

        # Telemetry / flight poller.
        def _poll():
            model_name = os.path.basename(str(opts.get("model", "-")))
            while not stop_event.is_set():
                self._emit_telemetry(drone, hooks)
                with lock:
                    fr = frames.get(key)
                hooks.emit_flight({
                    "connection": "CONNECTED",
                    "phase": state.get("phase", "-"),
                    "elapsed": round(time.time() - t0, 1),
                    "detected": bool(adet.latest_detections),
                    "model": model_name,
                    "inference_ms": round(adet.avg_inference_time),
                    "resolution": f"{fr.width}x{fr.height}" if fr is not None else "-",
                    "frame": fr.frame_number if fr is not None else 0,
                })
                time.sleep(0.3)

        threading.Thread(target=_poll, daemon=True).start()

        # Wait for the first frame so the mission's geometry is real.
        deadline = time.time() + 10.0
        while time.time() < deadline and not stop_event.is_set():
            with lock:
                if frames.get(key) is not None:
                    break
            time.sleep(0.1)

        try:
            demo.run_mission(drone, adet, frames, key, lock, state, opts,
                             stop_event, hooks.emit_log)
        finally:
            stop_event.set()
            for fn in (adet.stop, stream.stop, drone.disconnect):
                try:
                    fn()
                except Exception:  # noqa: BLE001
                    pass
            hooks.emit_flight({"connection": "disconnected", "phase": state.get("phase", "-")})
            hooks.emit_log("disconnected.")

    @staticmethod
    def _emit_telemetry(drone, hooks) -> None:
        t: Dict[str, Any] = {}
        try:
            t["battery"] = drone.get_battery()
        except Exception:  # noqa: BLE001
            pass
        try:
            t["height_cm"] = drone._server.get_plane_distance()
        except Exception:  # noqa: BLE001
            pass
        try:
            o = drone.get_orientation()
            t.update(roll=round(o.roll, 1), pitch=round(o.pitch, 1), yaw=round(o.yaw, 1))
        except Exception:  # noqa: BLE001
            pass
        try:
            v = drone.get_velocity()
            t.update(vx=round(v.x, 1), vy=round(v.y, 1), vz=round(v.z, 1))
        except Exception:  # noqa: BLE001
            pass
        try:
            p = drone.get_position()
            t.update(pos_x=round(p.x, 1), pos_y=round(p.y, 1), pos_z=round(p.z, 1))
        except Exception:  # noqa: BLE001
            pass
        if t:
            hooks.emit_telemetry(t)
