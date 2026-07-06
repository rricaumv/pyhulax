#!/usr/bin/env python3
"""Four-drone square choreography demo.

Four drones start at the corners of a 60 cm x 60 cm square, all facing the same
"forward" direction, and perform a synchronized routine:

  1. Placed at the 4 corners of a 60x60 cm square, all facing forward.
  2. Each drone's video streams in its own window, arranged 2x2 to mirror the
     physical corner layout.
  3. All take off to 100 cm, then enable QR-mat localization (--qr, default on)
     so position holding uses the QR code mat.
  4. Each rotates to face diagonally outward toward its own corner
     (e.g. the bottom-left drone turns to face the bottom-left / south-west).
  5. Each flies straight out along that diagonal until the four drones sit at
     the corners of a 100x100 cm square, then hovers.
  6. All four fly clockwise along 2 edges of the 100x100 square (--loop-sides),
     one edge at a time (each on a different edge, so no collisions), ending at
     the diagonally opposite corner.
  7. Each rotates back to the original "forward" heading.
  8. All land at the corner they finished on.

With the default 2 edges each drone finishes at, and lands on, the diagonally
opposite corner of the 100 cm square, so the formation swaps across the
diagonals. Use --loop-sides 4 for a full loop that returns each drone to its own
starting corner before landing.

The drones are synchronized phase-by-phase with a barrier, so they move together.

Coordinate convention: forward = +Y (north), right = +X (east). rotate(+deg) is
counter-clockwise (left); rotate(-deg) is clockwise (right).

Usage (order is bottom-left bottom-right top-left top-right):

    python examples/swarm_square_demo.py \
        --ips 192.168.1.58 192.168.1.70 192.168.1.71 192.168.1.72 \
        --ids 1 2 3 4 --video

    # Validate geometry/wiring without any hardware:
    python examples/swarm_square_demo.py --check
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

from pyhulax import DroneAPI  # noqa: E402
from pyhulax.core import Direction, LEDConfig, LEDMode  # noqa: E402
from pyhulax.core.exceptions import DroneError, DroneConnectionError  # noqa: E402


# --------------------------------------------------------------------------- #
# Geometry: corner order of --ips/--ids and each corner's parameters.
# out  = rotation (deg) from "forward" to face diagonally outward (+ = CCW/left)
# grid = (row, col) window cell so the 2x2 video layout mirrors the field.
# --------------------------------------------------------------------------- #
# out       = rotation (deg) from "forward" to face diagonally outward (+ = CCW/left)
# grid      = (row, col) window cell so the 2x2 video layout mirrors the field
# cw_start  = compass heading (0=N/forward, 90=E, 180=S, 270=W) of the first edge
#             this corner travels along when going clockwise round the perimeter
CORNER_ORDER = ["BL", "BR", "TL", "TR"]
CORNER_META = {
    "BL": {"name": "bottom-left",  "out": +135, "grid": (1, 0), "cw_start": 0},    # face SW; first edge N
    "BR": {"name": "bottom-right", "out": -135, "grid": (1, 1), "cw_start": 270},  # face SE; first edge W
    "TL": {"name": "top-left",     "out": +45,  "grid": (0, 0), "cw_start": 90},   # face NW; first edge E
    "TR": {"name": "top-right",    "out": -45,  "grid": (0, 1), "cw_start": 180},  # face NE; first edge S
}

LED = LEDConfig(r=0, g=255, b=0, mode=LEDMode.CONSTANT)

_COMPASS = {0: "N", 90: "E", 180: "S", 270: "W"}


class _Aborted(Exception):
    """Raised inside a worker when the routine is aborted (peer failure)."""


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

    def fail(self, corner: str, exc: BaseException) -> None:
        self.errors[corner] = exc
        self._abort.set()
        self._barrier.abort()


def _rotate_to(drone: DroneAPI, state: dict, target_heading: int) -> None:
    """Rotate to an absolute compass heading via the shorter direction.

    Headings: 0 = North (the original "forward"), 90 = East, 180 = South,
    270 = West. rotate(+deg) is CCW/left, rotate(-deg) is CW/right, so a
    clockwise (right) turn increases the compass heading.
    """
    target_heading %= 360
    cw = (target_heading - state["heading"]) % 360  # right-turn amount
    if cw == 0:
        pass
    elif cw <= 180:
        drone.rotate(-cw, led=LED)          # turn right (clockwise)
    else:
        drone.rotate(360 - cw, led=LED)     # turn left (shorter way)
    state["heading"] = target_heading


def _side_len_cm(outer_cm: float) -> int:
    """Length of one side of the outer square (firmware accepts 5..500 cm)."""
    return max(5, min(500, round(outer_cm)))


def diagonal_distance_cm(inner_cm: float, outer_cm: float) -> int:
    """Straight-line distance a corner drone flies from inner to outer square."""
    leg = (outer_cm - inner_cm) / 2.0        # per-axis displacement
    dist = round(math.hypot(leg, leg))       # along the diagonal
    return max(5, min(500, dist))            # firmware accepts 5..500 cm


# --------------------------------------------------------------------------- #
# Video: per-drone capture callback + a main-thread 2x2 grid renderer.
# All OpenCV GUI calls happen on the main thread (imshow/waitKey) for safety;
# the stream threads only stash the latest frame.
# --------------------------------------------------------------------------- #
def _start_capture(drone, corner, frames, lock, log):
    try:
        import pyhulax.video  # noqa: F401 - ensure the video extra is present
    except ImportError as exc:
        log(f"[{corner}] video unavailable - install 'pyhulax[video]' ({exc})")
        return None
    try:
        drone.set_video_stream(True)
        stream = drone.create_video_stream()

        def _cb(frame):
            try:
                with lock:
                    frames[corner] = frame.image
            except Exception:  # noqa: BLE001
                pass
            return frame

        stream.add_callback(_cb)
        stream.start()
        log(f"[{corner}] video streaming")
        return stream
    except Exception as exc:  # noqa: BLE001
        log(f"[{corner}] could not start video: {exc}")
        return None


def _grid_positions(specs, cell):
    cw, ch = cell
    gap, margin = 30, 20
    pos = {}
    for spec in specs:
        row, col = spec["grid"]
        pos[spec["corner"]] = (margin + col * (cw + gap), margin + row * (ch + gap))
    return pos


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
            for corner, img in snapshot.items():
                win = f"Drone {corner}"
                if corner not in created:
                    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(win, cw, ch)
                    x, y = positions.get(corner, (0, 0))
                    cv2.moveWindow(win, x, y)
                    created.add(corner)
                cv2.imshow(win, img)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Flight worker: the 9-step routine for one drone, barrier-synced per phase.
# --------------------------------------------------------------------------- #
def _worker(choreo, spec, drone, height, dist, side_len, sides, qr):
    corner = spec["corner"]

    def log(msg):
        print(f"[{corner} id{spec['id']}] {msg}", flush=True)

    try:
        choreo.sync()  # everyone connected + video started

        log(f"3. takeoff to {height} cm")
        drone.takeoff(height_cm=height, led=LED)
        if qr:
            log("   enable QR localization (using the QR mat)")
            try:
                drone.set_qr_localization(True)
                drone.hover(1)  # let the down-camera acquire the mat grid
            except Exception as exc:  # noqa: BLE001
                log(f"   QR enable failed: {exc}")
        choreo.sync()

        log(f"4. rotate {spec['out']:+d} deg to face {spec['name']} (outward)")
        drone.rotate(spec["out"], led=LED)
        choreo.sync()

        log(f"5. fly out {dist} cm to the outer square corner")
        drone.move(Direction.FORWARD, dist, led=LED)
        drone.hover(1)
        choreo.sync()

        # 6. Fly clockwise along `sides` edges of the outer square. All four
        #    drones advance one edge at a time (each on a different edge, so no
        #    collisions). With sides=2 each drone ends at the diagonally
        #    opposite corner.
        state = {"heading": (-spec["out"]) % 360}  # currently facing outward
        for side in range(sides):
            heading = (spec["cw_start"] + 90 * side) % 360
            log(f"6.{side + 1} face {_COMPASS[heading]} and fly {side_len} cm along the edge")
            _rotate_to(drone, state, heading)
            choreo.sync()
            drone.move(Direction.FORWARD, side_len, led=LED)
            choreo.sync()
        # 6 done: each drone is now at the corner `sides` steps clockwise from
        # its start (the diagonally opposite corner for sides=2). The drones
        # land here, at their new corners.

        log("7. rotate to the original forward heading")
        _rotate_to(drone, state, 0)
        choreo.sync()

        log("8. land at the new corner")
        drone.land(led=LED)
    except _Aborted:
        log("aborted (a peer failed) - landing")
    except Exception as exc:  # noqa: BLE001
        log(f"ERROR: {exc} - landing")
        choreo.fail(corner, exc)
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
    for i, corner in enumerate(CORNER_ORDER):
        meta = CORNER_META[corner]
        specs.append({"corner": corner, "ip": ips[i], "id": ids[i], **meta})
    return specs


def run(specs, *, height, inner, outer, connect_timeout, video, cell, sides, qr):
    dist = diagonal_distance_cm(inner, outer)
    side_len = _side_len_cm(outer)
    print(f"=== 4-drone square demo: {inner}cm -> {outer}cm square, diagonal leg "
          f"{dist}cm, perimeter side {side_len}cm, {sides} edge(s) clockwise, "
          f"QR={'on' if qr else 'off'}, height {height}cm ===")

    # --- connect all drones concurrently ---
    drones: dict[str, DroneAPI] = {}
    connect_errors: dict[str, BaseException] = {}

    def _connect(spec):
        d = DroneAPI(drone_id=spec["id"])
        drones[spec["corner"]] = d
        try:
            print(f"[{spec['corner']} id{spec['id']}] connecting to {spec['ip']} ...", flush=True)
            d.connect(spec["ip"], timeout=connect_timeout)
        except DroneConnectionError as exc:
            connect_errors[spec["corner"]] = exc

    cths = [threading.Thread(target=_connect, args=(s,)) for s in specs]
    for t in cths:
        t.start()
    for t in cths:
        t.join()

    if connect_errors:
        for corner, exc in connect_errors.items():
            print(f"[{corner}] CONNECT FAILED: {exc}")
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
                drones[spec["corner"]], spec["corner"], frames, frames_lock,
                log=lambda m: print(m, flush=True),
            )
            if st is not None:
                streams[spec["corner"]] = st

    # --- run the choreography ---
    choreo = Choreo(len(specs))
    workers = [
        threading.Thread(
            target=_worker,
            args=(choreo, spec, drones[spec["corner"]], height, dist, side_len, sides, qr),
            name=spec["corner"],
        )
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

    print("=== 4-drone square demo complete ===")
    if choreo.errors:
        raise SystemExit(f"{len(choreo.errors)} drone(s) errored: {list(choreo.errors)}")


def check(specs, *, height, inner, outer, cell, sides, qr):
    """Print the planned geometry/wiring without any hardware."""
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    dist = diagonal_distance_cm(inner, outer)
    side_len = _side_len_cm(outer)
    print(f"square {inner}cm -> {outer}cm, height {height}cm, diagonal leg {dist}cm, "
          f"perimeter side {side_len}cm, {sides} edge(s) clockwise, QR={'on' if qr else 'off'}")
    positions = _grid_positions(specs, cell)
    # Corner reached after `sides` clockwise steps (for the end/land summary).
    cw_seq = ["TL", "TR", "BR", "BL"]  # clockwise corner order
    for spec in specs:
        idx = cw_seq.index(spec["corner"])
        ends_at = cw_seq[(idx + sides) % 4]
        port = 9000 + spec["id"] * 2  # RTP port each drone's video would use
        print(f"  {spec['corner']} ({spec['name']:>12}) id={spec['id']} ip={spec['ip']:<15} "
              f"face-out={spec['out']:+4d}deg  cw-start={_COMPASS[spec['cw_start']]}  "
              f"ends@{ends_at}  window@{positions[spec['corner']]}  rtp:{port}")
    print("=== check passed ===")


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ips", nargs=4, metavar=("BL", "BR", "TL", "TR"),
                   help="4 drone IPs in order: bottom-left bottom-right top-left top-right")
    p.add_argument("--ids", nargs=4, type=int, default=[1, 2, 3, 4],
                   metavar=("BL", "BR", "TL", "TR"),
                   help="4 drone ids in the same order (default 1 2 3 4)")
    p.add_argument("--height", type=int, default=100, help="Takeoff height in cm (default 100)")
    p.add_argument("--inner", type=float, default=60.0, help="Inner square side in cm (default 60)")
    p.add_argument("--outer", type=float, default=100.0, help="Outer square side in cm (default 100)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="Seconds to wait for each drone's heartbeat (default 15)")
    p.add_argument("--loop-sides", type=int, default=2, choices=range(0, 5),
                   metavar="{0..4}",
                   help="Edges of the outer square to fly clockwise in step 6 "
                        "(default 2 = end at the diagonally opposite corner; "
                        "4 = full loop back to the start corner)")
    p.add_argument("--qr", action=argparse.BooleanOptionalAction, default=True,
                   help="Enable QR-mat localization after takeoff (default: enabled)")
    p.add_argument("--cell", nargs=2, type=int, default=[480, 360], metavar=("W", "H"),
                   help="Video window size in px for the 2x2 grid (default 480 360)")
    p.add_argument("--video", action="store_true",
                   help="Stream each drone in its own 2x2-arranged window "
                        "(requires 'pip install pyhulax[video]')")
    p.add_argument("--check", action="store_true",
                   help="Print planned geometry/wiring; no hardware, no flight")
    args = p.parse_args(argv)

    ips = args.ips or ["0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0"]
    if not args.check and args.ips is None:
        p.error("--ips is required (4 IPs) unless using --check")

    specs = build_specs(ips, args.ids)
    cell = (args.cell[0], args.cell[1])

    if args.check:
        check(specs, height=args.height, inner=args.inner, outer=args.outer, cell=cell,
              sides=args.loop_sides, qr=args.qr)
        return

    run(specs, height=args.height, inner=args.inner, outer=args.outer,
        connect_timeout=args.connect_timeout, video=args.video, cell=cell,
        sides=args.loop_sides, qr=args.qr)


if __name__ == "__main__":
    main()
