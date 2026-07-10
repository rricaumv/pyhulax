#!/usr/bin/env python3
"""Four-drone vertical-weave demo (with 2x2 video windows).

Take-off positions on the QR mat (coordinates in cm):

    drone 0  bottom-left   (25,  25)     drone 1  bottom-right (125,  25)
    drone 2  top-left      (25, 125)     drone 3  top-right    (125, 125)

Drones 0 and 1 are the "front" pair (group A); drones 2 and 3 are the "back"
pair (group B). The two pairs swap places twice, always separated in altitude
so their flight paths never collide:

  1.  All take off to 100 cm.
  2.  Hover 3 s.
  3.  Group A descend to 60 cm; group B ascend to 140 cm.
  4.  Hover 2 s.
  5.  Group A fly forward 100 cm; group B fly backward 100 cm (no yaw).
  6.  Group A ascend to 140 cm; group B descend to 60 cm.
  7.  Hover 1 s.
  8.  Group A fly backward 100 cm; group B fly forward 100 cm (no yaw).
  9.  Hover 1 s.
  10. All return to 100 cm.
  11. Hover 3 s.
  12. Land.

The 80 cm altitude gap during steps 5 and 8 is what keeps the crossing pairs
apart. The drones never yaw, so "forward"/"backward" stay along each drone's
original heading. QR-mat localization is enabled after takeoff (--qr).

Video (optional) streams each drone in its own window, arranged 2x2 to mirror
the mat layout. Order of --ips/--ids is drone 0,1,2,3 (BL, BR, TL, TR).

Usage:

    python examples/swarm_vertical_weave_demo.py \
        --ips 192.168.1.58 192.168.1.70 192.168.1.71 192.168.1.72 \
        --ids 0 1 2 3 --video

    # Print the planned heights/wiring without hardware:
    python examples/swarm_vertical_weave_demo.py --check
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
import threading  # noqa: E402

from pyhulax import DroneAPI  # noqa: E402
from pyhulax.core import Direction, LEDConfig, LEDMode  # noqa: E402
from pyhulax.core.exceptions import DroneError, DroneConnectionError  # noqa: E402


# --------------------------------------------------------------------------- #
# Drone layout. group A = front pair (0,1), group B = back pair (2,3).
# grid = (row, col) video-window cell mirroring the mat (row 0 = top).
# --------------------------------------------------------------------------- #
DRONES = [
    {"idx": 0, "label": "BL", "pos": (25, 25),   "group": "A", "grid": (1, 0)},
    {"idx": 1, "label": "BR", "pos": (125, 25),  "group": "A", "grid": (1, 1)},
    {"idx": 2, "label": "TL", "pos": (25, 125),  "group": "B", "grid": (0, 0)},
    {"idx": 3, "label": "TR", "pos": (125, 125), "group": "B", "grid": (0, 1)},
]

CRUISE_CM = 100      # baseline height
LOW_CM = 60          # low layer
HIGH_CM = 140        # high layer
RUN_CM = 100         # forward/backward run distance

LED = LEDConfig(r=0, g=255, b=0, mode=LEDMode.CONSTANT)


class _Aborted(Exception):
    """Raised in a worker when the routine is aborted (peer failure)."""


class Choreo:
    """Phase barrier shared by the drone workers, with cooperative abort."""

    def __init__(self, parties: int, phase_timeout: float = 120.0):
        self._barrier = threading.Barrier(parties)
        self._abort = threading.Event()
        self._timeout = phase_timeout
        self.errors: dict[str, BaseException] = {}

    def sync(self) -> None:
        if self._abort.is_set():
            raise _Aborted()
        try:
            self._barrier.wait(timeout=self._timeout)
        except threading.BrokenBarrierError:
            raise _Aborted()

    def fail(self, who: str, exc: BaseException) -> None:
        self.errors[who] = exc
        self._abort.set()
        self._barrier.abort()


def _set_height(drone: DroneAPI, state: dict, target: int) -> None:
    """Move vertically to an absolute target height by the tracked delta."""
    delta = target - state["h"]
    if delta > 0:
        drone.move(Direction.UP, delta, led=LED)
    elif delta < 0:
        drone.move(Direction.DOWN, -delta, led=LED)
    state["h"] = target


# --------------------------------------------------------------------------- #
# Video: per-drone capture + main-thread 2x2 grid renderer (OpenCV on the main
# thread; stream threads only stash the latest frame).
# --------------------------------------------------------------------------- #
def _start_capture(drone, key, frames, lock, log):
    try:
        import pyhulax.video  # noqa: F401 - ensure the video extra is present
    except ImportError as exc:
        log(f"[{key}] video unavailable - install 'pyhulax[video]' ({exc})")
        return None
    try:
        drone.set_video_stream(True)
        stream = drone.create_video_stream()

        def _cb(frame):
            try:
                with lock:
                    frames[key] = frame.image
            except Exception:  # noqa: BLE001
                pass
            return frame

        stream.add_callback(_cb)
        stream.start()
        log(f"[{key}] video streaming")
        return stream
    except Exception as exc:  # noqa: BLE001
        log(f"[{key}] could not start video: {exc}")
        return None


def _grid_positions(specs, cell):
    cw, ch = cell
    gap, margin = 30, 20
    return {
        s["label"]: (margin + s["grid"][1] * (cw + gap), margin + s["grid"][0] * (ch + gap))
        for s in specs
    }


def _display_loop(frames, lock, workers, specs, cell):
    try:
        import cv2
    except ImportError:
        for w in workers:
            w.join()
        return
    positions = _grid_positions(specs, cell)
    cw, ch = cell
    created: set[str] = set()
    try:
        while any(w.is_alive() for w in workers):
            with lock:
                snapshot = dict(frames)
            for key, img in snapshot.items():
                win = f"Drone {key}"
                if key not in created:
                    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(win, cw, ch)
                    x, y = positions.get(key, (0, 0))
                    cv2.moveWindow(win, x, y)
                    created.add(key)
                cv2.imshow(win, img)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Flight worker: the 12-step routine for one drone, barrier-synced per phase.
# --------------------------------------------------------------------------- #
def _worker(choreo, spec, drone, qr):
    key = spec["label"]
    group_a = spec["group"] == "A"

    def log(msg):
        print(f"[drone{spec['idx']} {key}] {msg}", flush=True)

    # Per-group vertical targets and horizontal directions.
    step3 = LOW_CM if group_a else HIGH_CM
    step6 = HIGH_CM if group_a else LOW_CM
    run5 = Direction.FORWARD if group_a else Direction.BACK
    run8 = Direction.BACK if group_a else Direction.FORWARD

    state = {"h": 0}
    try:
        choreo.sync()  # everyone connected + video started

        log("1. takeoff to 100 cm")
        drone.takeoff(height_cm=CRUISE_CM, led=LED)
        state["h"] = CRUISE_CM
        if qr:
            log("   enable QR localization (using the QR mat)")
            try:
                drone.set_qr_localization(True)
                drone.hover(1)
            except Exception as exc:  # noqa: BLE001
                log(f"   QR enable failed: {exc}")
        choreo.sync()

        log("2. hover 3 s")
        drone.hover(3)
        choreo.sync()

        log(f"3. change height to {step3} cm")
        _set_height(drone, state, step3)
        choreo.sync()

        log("4. hover 2 s")
        drone.hover(2)
        choreo.sync()

        log(f"5. fly {run5.name.lower()} {RUN_CM} cm (no yaw)")
        drone.move(run5, RUN_CM, led=LED)
        choreo.sync()

        log(f"6. change height to {step6} cm")
        _set_height(drone, state, step6)
        choreo.sync()

        log("7. hover 1 s")
        drone.hover(1)
        choreo.sync()

        log(f"8. fly {run8.name.lower()} {RUN_CM} cm (no yaw)")
        drone.move(run8, RUN_CM, led=LED)
        choreo.sync()

        log("9. hover 1 s")
        drone.hover(1)
        choreo.sync()

        log(f"10. return to {CRUISE_CM} cm")
        _set_height(drone, state, CRUISE_CM)
        choreo.sync()

        log("11. hover 3 s")
        drone.hover(3)
        choreo.sync()

        log("12. land")
        drone.land(led=LED)
    except _Aborted:
        log("aborted (a peer failed) - landing")
    except Exception as exc:  # noqa: BLE001
        log(f"ERROR: {exc} - landing")
        choreo.fail(key, exc)
    finally:
        try:
            if drone.is_connected:
                drone.land()
        except DroneError:
            pass


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def build_specs(ips, ids):
    specs = []
    for i, base in enumerate(DRONES):
        specs.append({**base, "ip": ips[i], "id": ids[i]})
    return specs


def run(specs, *, connect_timeout, video, cell, qr):
    print(f"=== 4-drone vertical-weave demo: cruise {CRUISE_CM}cm, "
          f"low {LOW_CM}cm, high {HIGH_CM}cm, run {RUN_CM}cm, "
          f"QR={'on' if qr else 'off'} ===")

    # --- connect all drones concurrently ---
    drones: dict[int, DroneAPI] = {}
    connect_errors: dict[str, BaseException] = {}

    def _connect(spec):
        d = DroneAPI(drone_id=spec["id"])
        drones[spec["idx"]] = d
        try:
            print(f"[drone{spec['idx']} {spec['label']}] connecting to {spec['ip']} ...", flush=True)
            d.connect(spec["ip"], timeout=connect_timeout)
        except DroneConnectionError as exc:
            connect_errors[spec["label"]] = exc

    cths = [threading.Thread(target=_connect, args=(s,)) for s in specs]
    for t in cths:
        t.start()
    for t in cths:
        t.join()

    if connect_errors:
        for who, exc in connect_errors.items():
            print(f"[{who}] CONNECT FAILED: {exc}")
        for d in drones.values():
            try:
                d.disconnect()
            except Exception:  # noqa: BLE001
                pass
        raise SystemExit("Aborting: all four drones must connect for the routine.")

    # --- start video capture (optional) ---
    frames: dict[str, object] = {}
    frames_lock = threading.Lock()
    streams: dict[str, object] = {}
    if video:
        for spec in specs:
            st = _start_capture(
                drones[spec["idx"]], spec["label"], frames, frames_lock,
                log=lambda m: print(m, flush=True),
            )
            if st is not None:
                streams[spec["label"]] = st

    # --- run the choreography ---
    choreo = Choreo(len(specs))
    workers = [
        threading.Thread(target=_worker, args=(choreo, spec, drones[spec["idx"]], qr),
                         name=spec["label"])
        for spec in specs
    ]
    for w in workers:
        w.start()

    if video and streams:
        _display_loop(frames, frames_lock, workers, specs, cell)  # main thread

    for w in workers:
        w.join()

    # --- cleanup ---
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

    print("=== vertical-weave demo complete ===")
    if choreo.errors:
        raise SystemExit(f"{len(choreo.errors)} drone(s) errored: {list(choreo.errors)}")


def check(specs, *, cell, qr):
    """Print the planned heights/wiring without any hardware."""
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    print(f"cruise {CRUISE_CM}cm, low {LOW_CM}cm, high {HIGH_CM}cm, run {RUN_CM}cm, "
          f"QR={'on' if qr else 'off'}")
    positions = _grid_positions(specs, cell)
    for spec in specs:
        a = spec["group"] == "A"
        heights = f"100->{LOW_CM if a else HIGH_CM}->{HIGH_CM if a else LOW_CM}->100"
        runs = "fwd/back" if a else "back/fwd"
        port = 9000 + spec["id"] * 2
        print(f"  drone{spec['idx']} {spec['label']} at {spec['pos']} group {spec['group']} "
              f"id={spec['id']} ip={spec['ip']:<15} heights={heights} runs={runs} "
              f"window@{positions[spec['label']]} rtp:{port}")
    print("=== check passed ===")


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ips", nargs=4, metavar=("D0", "D1", "D2", "D3"),
                   help="4 drone IPs in order: drone 0 (BL) 1 (BR) 2 (TL) 3 (TR)")
    p.add_argument("--ids", nargs=4, type=int, default=[0, 1, 2, 3],
                   metavar=("D0", "D1", "D2", "D3"),
                   help="4 drone ids in the same order (default 0 1 2 3)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="Seconds to wait for each drone's heartbeat (default 15)")
    p.add_argument("--qr", action=argparse.BooleanOptionalAction, default=True,
                   help="Enable QR-mat localization after takeoff (default: enabled)")
    p.add_argument("--cell", nargs=2, type=int, default=[480, 360], metavar=("W", "H"),
                   help="Video window size in px for the 2x2 grid (default 480 360)")
    p.add_argument("--video", action="store_true",
                   help="Stream each drone in its own 2x2-arranged window "
                        "(requires 'pip install pyhulax[video]')")
    p.add_argument("--check", action="store_true",
                   help="Print planned heights/wiring; no hardware, no flight")
    args = p.parse_args(argv)

    ips = args.ips or ["0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0"]
    if not args.check and args.ips is None:
        p.error("--ips is required (4 IPs) unless using --check")

    specs = build_specs(ips, args.ids)
    cell = (args.cell[0], args.cell[1])

    if args.check:
        check(specs, cell=cell, qr=args.qr)
        return

    run(specs, connect_timeout=args.connect_timeout, video=args.video, cell=cell, qr=args.qr)


if __name__ == "__main__":
    main()
