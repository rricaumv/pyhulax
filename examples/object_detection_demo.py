#!/usr/bin/env python3
"""YOLO object-detection demo for one or more drones (video only, no flight).

Connects to each drone, starts its video stream, runs a YOLO detector on every
frame, draws the boxes, and shows each drone in its own window (auto-arranged in
a grid). The drones do NOT take off - this is purely the camera + detection
pipeline, so you can point a drone at things and watch detections live.

Pipeline per drone:  YOLODetector -> DrawDetections -> window

Requires the video + YOLO deps:  pip install "pyhulax[video]" ultralytics
(The first run downloads the model file, e.g. yolov8n.pt, if not present.)

Usage:

    # One drone
    python examples/object_detection_demo.py --ips 192.168.1.58

    # Several drones, bigger model, higher confidence
    python examples/object_detection_demo.py \
        --ips 192.168.1.58 192.168.1.70 --model yolov8s.pt --confidence 0.4

    # Only detect people and cars (COCO class ids 0 and 2)
    python examples/object_detection_demo.py --ips 192.168.1.58 --classes 0 2

Press 'q' in any window (or Ctrl-C) to stop.
"""

from __future__ import annotations

# --------------------------------------------------------------------------- #
# Bootstrap: force-load the in-repo pyhulax, not an installed copy.
# --------------------------------------------------------------------------- #
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import pyhulax  # noqa: E402

_resolved = os.path.dirname(os.path.abspath(pyhulax.__file__))
if os.path.normcase(_resolved) != os.path.normcase(os.path.join(_REPO_ROOT, "pyhulax")):
    raise SystemExit(f"Refusing to run: pyhulax resolved to {_resolved}, not the repo copy.")

import argparse  # noqa: E402
import math  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

from pyhulax import DroneAPI  # noqa: E402
from pyhulax.core.exceptions import DroneConnectionError  # noqa: E402


def _grid_dims(n: int) -> tuple[int, int]:
    """Rows, cols for an n-window grid (as square as possible)."""
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    return rows, cols


def _window_positions(n: int, cell: tuple[int, int]) -> list[tuple[int, int]]:
    cw, ch = cell
    gap, margin = 30, 20
    _, cols = _grid_dims(n)
    pos = []
    for i in range(n):
        r, c = divmod(i, cols)
        pos.append((margin + c * (cw + gap), margin + r * (ch + gap)))
    return pos


# --------------------------------------------------------------------------- #
# Detection stream: attach YOLODetector -> DrawDetections -> capture callback.
# Each drone gets its own detector instance (ultralytics models are not safe to
# call concurrently from multiple threads).
# --------------------------------------------------------------------------- #
def _start_detection(drone, key, model_path, confidence, classes, imgsz,
                     frames, lock, log):
    try:
        from pyhulax.video import YOLODetector, DrawDetections
    except ImportError as exc:
        log(f"[{key}] detection unavailable: {exc}\n"
            f"      install with: pip install 'pyhulax[video]' ultralytics")
        return None
    try:
        drone.set_video_stream(True)
        stream = drone.create_video_stream()
        detector = YOLODetector(
            model_path=model_path,
            confidence=confidence,
            classes=classes,
            imgsz=imgsz,
        )

        def _capture(frame):
            try:
                with lock:
                    frames[key] = frame  # annotated image + .detections
            except Exception:  # noqa: BLE001
                pass
            return frame

        stream.add_callback(detector)          # sets frame.detections
        stream.add_callback(DrawDetections())  # draws boxes onto frame.image
        stream.add_callback(_capture)          # stash for the display thread
        stream.start()
        log(f"[{key}] detection stream started (model={model_path})")
        return stream
    except Exception as exc:  # noqa: BLE001
        log(f"[{key}] could not start detection stream: {exc}")
        return None


def _display_loop(keys, frames, lock, cell, stop_event):
    """Main-thread window loop. Returns when 'q' pressed or stop_event set."""
    try:
        import cv2
    except ImportError:
        # Headless fallback: periodically log detections until stopped.
        while not stop_event.is_set():
            with lock:
                snap = {k: frames.get(k) for k in keys}
            for k, fr in snap.items():
                if fr is not None and getattr(fr, "detections", None):
                    labels = ", ".join(sorted({d.label for d in fr.detections}))
                    print(f"[{k}] {len(fr.detections)} detections: {labels}", flush=True)
            time.sleep(2.0)
        return

    positions = _window_positions(len(keys), cell)
    cw, ch = cell
    created: dict[str, bool] = {}
    while not stop_event.is_set():
        with lock:
            snap = {k: frames.get(k) for k in keys}
        for i, k in enumerate(keys):
            fr = snap.get(k)
            if fr is None:
                continue
            win = f"Drone {k}"
            if k not in created:
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(win, cw, ch)
                x, y = positions[i]
                cv2.moveWindow(win, x, y)
                created[k] = True
            cv2.imshow(win, fr.image)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    try:
        cv2.destroyAllWindows()
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def run(specs, *, connect_timeout, model, confidence, classes, imgsz, cell, duration):
    print(f"=== object detection: {len(specs)} drone(s), model={model}, "
          f"conf={confidence}, classes={classes or 'all'} ===")

    drones: dict[str, DroneAPI] = {}
    connect_errors: dict[str, BaseException] = {}

    def _connect(spec):
        d = DroneAPI(drone_id=spec["id"])
        drones[spec["key"]] = d
        try:
            print(f"[{spec['key']}] connecting to {spec['ip']} ...", flush=True)
            d.connect(spec["ip"], timeout=connect_timeout)
        except DroneConnectionError as exc:
            connect_errors[spec["key"]] = exc

    cths = [threading.Thread(target=_connect, args=(s,)) for s in specs]
    for t in cths:
        t.start()
    for t in cths:
        t.join()

    if connect_errors:
        for key, exc in connect_errors.items():
            print(f"[{key}] CONNECT FAILED: {exc}")
        for d in drones.values():
            try:
                d.disconnect()
            except Exception:  # noqa: BLE001
                pass
        raise SystemExit("Aborting: could not connect to all drones.")

    frames: dict[str, object] = {}
    lock = threading.Lock()
    streams: dict[str, object] = {}
    for spec in specs:
        st = _start_detection(
            drones[spec["key"]], spec["key"], model, confidence, classes, imgsz,
            frames, lock, log=lambda m: print(m, flush=True),
        )
        if st is not None:
            streams[spec["key"]] = st

    if not streams:
        for d in drones.values():
            d.disconnect()
        raise SystemExit("No detection streams started (missing video/YOLO deps?).")

    stop_event = threading.Event()
    timer = None
    if duration:
        timer = threading.Timer(duration, stop_event.set)
        timer.daemon = True
        timer.start()

    print("Streaming + detecting. Press 'q' in a window or Ctrl-C to stop.")
    try:
        _display_loop([s["key"] for s in specs], frames, lock, cell, stop_event)
    except KeyboardInterrupt:
        print("\nInterrupted - stopping.")
    finally:
        if timer is not None:
            timer.cancel()
        for st in streams.values():
            try:
                st.stop()
            except Exception:  # noqa: BLE001
                pass
        for d in drones.values():
            try:
                d.disconnect()
            except Exception:  # noqa: BLE001
                pass
    print("=== object detection demo complete ===")


def check(specs, *, model, confidence, classes, cell):
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    print(f"model={model} conf={confidence} classes={classes or 'all'}")
    positions = _window_positions(len(specs), cell)
    rows, cols = _grid_dims(len(specs))
    print(f"window grid: {rows}x{cols}")
    for i, spec in enumerate(specs):
        port = 9000 + spec["id"] * 2
        print(f"  {spec['key']} id={spec['id']} ip={spec['ip']:<15} "
              f"window@{positions[i]} rtp:{port}")
    # Report whether the detection deps are importable (no model download).
    try:
        import ultralytics  # noqa: F401
        print("ultralytics: available")
    except ImportError:
        print("ultralytics: NOT installed (pip install ultralytics)")
    print("=== check passed ===")


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ips", nargs="+", metavar="IP",
                   help="One or more drone IPs")
    p.add_argument("--ids", nargs="+", type=int, metavar="ID",
                   help="Drone ids, one per IP (default 0 1 2 ...)")
    p.add_argument("--model", default="yolov8n.pt",
                   help="YOLO model path/name (default yolov8n.pt)")
    p.add_argument("--confidence", type=float, default=0.25,
                   help="Detection confidence threshold 0-1 (default 0.25)")
    p.add_argument("--classes", nargs="+", type=int, default=None, metavar="ID",
                   help="Only detect these class ids (e.g. COCO: 0=person 2=car)")
    p.add_argument("--imgsz", type=int, default=640,
                   help="YOLO inference image size (default 640)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="Seconds to wait for each drone's heartbeat (default 15)")
    p.add_argument("--duration", type=float, default=None,
                   help="Stop after this many seconds (default: run until 'q'/Ctrl-C)")
    p.add_argument("--cell", nargs=2, type=int, default=[640, 480], metavar=("W", "H"),
                   help="Window size in px (default 640 480)")
    p.add_argument("--check", action="store_true",
                   help="Print planned wiring and dependency status; no hardware")
    args = p.parse_args(argv)

    ips = args.ips or ["0.0.0.0"]
    if not args.check and args.ips is None:
        p.error("--ips is required (one or more) unless using --check")

    ids = args.ids if args.ids is not None else list(range(len(ips)))
    if len(ids) != len(ips):
        p.error(f"got {len(ips)} ip(s) but {len(ids)} id(s); they must match")

    specs = [{"key": f"D{i}", "ip": ips[i], "id": ids[i]} for i in range(len(ips))]
    cell = (args.cell[0], args.cell[1])

    if args.check:
        check(specs, model=args.model, confidence=args.confidence,
              classes=args.classes, cell=cell)
        return

    run(specs, connect_timeout=args.connect_timeout, model=args.model,
        confidence=args.confidence, classes=args.classes, imgsz=args.imgsz,
        cell=cell, duration=args.duration)


if __name__ == "__main__":
    main()
