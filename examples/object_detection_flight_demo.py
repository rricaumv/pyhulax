#!/usr/bin/env python3
"""Object-detection *flight* demo for a single drone: find a target, center on it,
flash, and retrace the path home.

Unlike ``object_detection_demo.py`` (video only), this one flies. The mission:

  1. takeoff      single_fly_takeoff, then climb to a target ToF height
                  (get_plane_distance) with single_fly_up / single_fly_down
  2. search       single_fly_turnright(step) in steps until the detector
                  reports the target class (e.g. "tank")
  3. center       strafe single_fly_left/right/up/down until the target box
                  center matches the frame center
  4. LED flash    single_fly_lamplight(r, g, b, time, mode) for 5 s -
                  --led-mode flash (blink --led-rgb colour, 32), rainbow
                  (seven-colour cycle, 16), or cycle (R->G->B, 4)
  5. return home  every executed motion is recorded and retraced in reverse
                  (inverse move, reverse order), then single_fly_touchdown().
                  This avoids relying on the ambiguous absolute-coordinate APIs.

Detection runs off the video decode thread via pyhulax.video.AsyncDetector, so a
slow model never stalls the stream. The window shows the live boxes, a frame-center
crosshair, and the current mission phase.

Requires the video + YOLO deps:  pip install "pyhulax[video]" ultralytics

Note on the target class: stock YOLO/COCO models do NOT have a "tank" class. Use a
model trained on your target (``--model my_tank.pt``), or pass ``--target person``
(or any COCO label) to rehearse the whole flight with a stock model. ``--target any``
locks onto the largest detection of any class.

Usage:

    # Real flight: find a tank with a custom model, center, flash, come home
    python examples/object_detection_flight_demo.py \
        --ip 192.168.1.58 --id 1 --model tank_yolov8.pt --target tank

    # Rehearse with a stock model against a person, rainbow flash on find
    python examples/object_detection_flight_demo.py \
        --ip 192.168.1.58 --id 1 --target person --led-mode rainbow

    # Custom flash colour (green) - three 0-255 values
    python examples/object_detection_flight_demo.py \
        --ip 192.168.1.58 --id 1 --led-rgb 0 255 0

    # Print the plan + verify the retrace/inverse logic, no hardware
    python examples/object_detection_flight_demo.py --check

Press 'q' in the window (or Ctrl-C) to abort - the drone lands where it is.
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
import time  # noqa: E402
import traceback  # noqa: E402

from pyhulax import DroneAPI  # noqa: E402
from pyhulax.core import LEDMode  # noqa: E402
from pyhulax.core.exceptions import DroneConnectionError  # noqa: E402


# LED effect for the "target found" flash (step 4). single_fly_lamplight takes a
# raw mode byte; these are the SDK's LEDMode values.
LED_FLASH_MODES = {
    "flash": int(LEDMode.BLINK),          # 32 - blink the single --led-rgb colour
    "rainbow": int(LEDMode.SEVEN_COLOR),  # 16 - multi-colour rainbow cycle
    "cycle": int(LEDMode.RGB_CYCLE),      # 4  - cycle red -> green -> blue
}


# --------------------------------------------------------------------------- #
# Motion log: every executed move is recorded so it can be retraced in reverse
# (inverse move, reverse order). The drone never trusts an absolute position -
# it just undoes exactly what it did to get back home.
# --------------------------------------------------------------------------- #
class MotionLog:
    """Wraps the control server's relative-move commands and records them.

    Each helper issues a blocking single_fly_* command and appends (name,
    magnitude) to the log. retrace() replays the inverse of every move in reverse
    order, returning the drone to where it started.
    """

    # Every recordable move and the command that undoes it.
    _INVERSE = {
        "up": "down",
        "down": "up",
        "left": "right",
        "right": "left",
        "forward": "back",
        "back": "forward",
        "turnright": "turnleft",
        "turnleft": "turnright",
    }

    def __init__(self, server, led: int = 0, log=print):
        self._server = server
        self._led = led
        self._log = log
        self._moves: list[tuple[str, float]] = []

    def _do(self, name: str, magnitude: float) -> None:
        magnitude = round(magnitude)
        if magnitude <= 0:
            return
        fn = getattr(self._server, f"single_fly_{name}")
        fn(magnitude, self._led)  # blocking=True by default
        self._moves.append((name, magnitude))

    # Recorded relative moves --------------------------------------------------
    def up(self, cm: float) -> None:
        self._do("up", cm)

    def down(self, cm: float) -> None:
        self._do("down", cm)

    def left(self, cm: float) -> None:
        self._do("left", cm)

    def right(self, cm: float) -> None:
        self._do("right", cm)

    def forward(self, cm: float) -> None:
        self._do("forward", cm)

    def back(self, cm: float) -> None:
        self._do("back", cm)

    def turn_right(self, deg: float) -> None:
        self._do("turnright", deg)

    def turn_left(self, deg: float) -> None:
        self._do("turnleft", deg)

    # Retrace ------------------------------------------------------------------
    @property
    def moves(self) -> list[tuple[str, float]]:
        return list(self._moves)

    def plan_retrace(self) -> list[tuple[str, float]]:
        """The inverse sequence, without executing it (inverse move, reverse order)."""
        return [(self._INVERSE[name], mag) for name, mag in reversed(self._moves)]

    def retrace(self, stop=None) -> None:
        for name, magnitude in self.plan_retrace():
            if stop is not None and stop.is_set():
                self._log("retrace aborted (stop requested)")
                return
            fn = getattr(self._server, f"single_fly_{name}")
            fn(magnitude, self._led)
        self._moves.clear()


# --------------------------------------------------------------------------- #
# Detection helpers
# --------------------------------------------------------------------------- #
def _make_detector(model_path, confidence, classes, imgsz, log):
    try:
        from pyhulax.video import YOLODetector
    except ImportError as exc:
        raise SystemExit(
            f"detection unavailable: {exc}\n"
            f"  install with: pip install 'pyhulax[video]' ultralytics"
        )
    try:
        return YOLODetector(model_path=model_path, confidence=confidence,
                            classes=classes, imgsz=imgsz)
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"could not create detector: {exc}")


def _pick_target(dets, target: str):
    """Largest detection in `dets` whose label matches `target` (case-insensitive).

    `target` == "any"/"*" matches the largest detection of any class. Returns a
    Detection or None.
    """
    if not dets:
        return None
    want = target.lower()
    if want in ("any", "*"):
        candidates = dets
    else:
        candidates = [d for d in dets if d.label.lower() == want]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.bbox.area)


def _observe(adet, target, settle, fresh_timeout, current_frame_number, stop,
             retries=0):
    """Find the target using a detection computed *after* the drone settled.

    The drone's video is buffered and inference lags, so `latest_detections`
    right after a move can still describe the pre-move view. This settles the
    view, notes a freshly-decoded frame number, waits for the detector to catch
    up past it, and only then picks the target - so search/centering never act on
    a box from an orientation the drone has already left.

    `retries` re-observes (without moving) through transient dropouts - a single
    blurred/missed frame right after a strafe should not count as "target lost".
    """
    for attempt in range(retries + 1):
        if stop.is_set():
            return None
        if attempt == 0:
            time.sleep(settle)  # settle once after the move; retries just re-look
        baseline = current_frame_number()  # a frame decoded after the move settled
        dets = adet.wait_for_fresh_detection(baseline, timeout=fresh_timeout)
        det = _pick_target(dets, target)
        if det is not None:
            return det
    return None


# --------------------------------------------------------------------------- #
# Flight phases
# --------------------------------------------------------------------------- #
def climb_to_height(server, motion, target_cm, tol, step, settle, log, stop):
    """Nudge up/down until the ToF height (get_plane_distance) is within tol."""
    for _ in range(20):
        if stop.is_set():
            return
        dist = server.get_plane_distance()
        log(f"  ToF height = {dist} cm (target {target_cm})")
        err = target_cm - dist
        if abs(err) <= tol:
            return
        move = min(step, abs(err))
        if err > 0:
            motion.up(move)
        else:
            motion.down(move)
        time.sleep(settle)


def search_for_target(motion, adet, target, step_deg, settle, fresh_timeout,
                      current_frame_number, log, stop):
    """Turn right in steps until the target is detected (or a full turn elapses).

    After each blocking turn the drone is stationary; only then - once a
    post-turn frame has been detected - do we decide whether the target is in
    view, so we never lock onto a box seen mid-rotation.
    """
    if _observe(adet, target, settle, fresh_timeout, current_frame_number, stop):
        log("  target already in view")
        return True
    turned = 0
    while turned < 360 and not stop.is_set():
        motion.turn_right(step_deg)  # blocking: the drone stops before we look
        turned += step_deg
        det = _observe(adet, target, settle, fresh_timeout,
                       current_frame_number, stop)
        log(f"  turned {turned} deg -> {'FOUND' if det else 'no target'}")
        if det is not None:
            return True
    return False


def center_on_target(motion, adet, target, frame_size, max_step, min_step,
                     deadband_frac, settle, fresh_timeout, current_frame_number,
                     max_steps, retries, log, stop):
    """Strafe until the target box center hits the frame center - robustly.

    Strategy (built to not lose the target):

    * One axis per iteration - correct the larger (normalized) error, then
      re-observe. Smaller net moves keep the target in frame vs. diagonal jumps.
    * Adaptive gain - unknown cm-per-pixel is learned from each move (how many
      pixels the box shifted per cm strafed) and used to size the next move, so
      it converges without the overshoot that ejects the target near an edge.
      Until it's learned, a conservative calibration step is used.
    * Retry through dropouts - a single blurred/missed frame after a strafe is
      re-observed (see _observe retries), not treated as "lost".
    * Back-off recovery - if the target really leaves the frame after a strafe,
      the strafe is undone to bring it back before giving up.

    Every move (including recovery) is recorded by MotionLog, so the retrace-home
    step still undoes the net displacement.
    """
    w, h = frame_size
    cxf, cyf = w / 2.0, h / 2.0
    dbx, dby = w * deadband_frac, h * deadband_frac
    # pixels-per-cm gain learned per axis (None until measured).
    gain = {"x": None, "y": None}

    def observe():
        return _observe(adet, target, settle, fresh_timeout,
                        current_frame_number, stop, retries=retries)

    det = observe()
    if det is None:
        log("  lost target while centering (not visible at start)")
        return False

    for _ in range(max_steps):
        if stop.is_set():
            return False
        cx, cy = det.bbox.center
        ex, ey = cx - cxf, cy - cyf  # >0 => target right/below center
        log(f"  center err = ({ex:+.0f}, {ey:+.0f}) px  "
            f"deadband=({dbx:.0f}, {dby:.0f})  gain={gain}")
        if abs(ex) <= dbx and abs(ey) <= dby:
            log("  centered")
            return True

        # Correct the axis with the larger normalized error (one move / iter).
        axis = "x" if (abs(ex) / (w / 2.0)) >= (abs(ey) / (h / 2.0)) else "y"
        err = ex if axis == "x" else ey

        g = gain[axis]
        if g and g > 1e-6:
            step = abs(err) / g            # cm needed to null this pixel error
        else:
            step = 0.5 * max_step          # calibration move (gain unknown yet)
        step = max(min_step, min(max_step, step))

        # Issue the single strafe; remember how to undo it for recovery.
        if axis == "x":
            forward, undo = (motion.right, motion.left) if err > 0 else (motion.left, motion.right)
        else:
            forward, undo = (motion.down, motion.up) if err > 0 else (motion.up, motion.down)
        forward(step)
        moved_cm = round(step)

        new = observe()
        if new is None:
            log("  target lost after strafe - backing off to recover")
            undo(step)
            gain[axis] = None              # our model misfired; recalibrate
            new = observe()
            if new is None:
                log("  lost target while centering")
                return False
            det = new
            continue

        # Learn the gain: how many pixels did the box move per cm strafed?
        ncx, ncy = new.bbox.center
        nerr = (ncx - cxf) if axis == "x" else (ncy - cyf)
        delta = abs(err) - abs(nerr)       # >0 => moved toward center
        if moved_cm > 0 and delta > 2:
            observed = delta / moved_cm
            gain[axis] = observed if gain[axis] is None else 0.5 * gain[axis] + 0.5 * observed
        elif delta < -2:
            gain[axis] = None              # overshoot/drift: drop the stale gain
        det = new

    log("  centering hit max steps")
    return abs(det.bbox.center[0] - cxf) <= dbx and abs(det.bbox.center[1] - cyf) <= dby


def flash_led(server, r, g, b, seconds, mode, log):
    """single_fly_lamplight for `seconds` using the chosen effect.

    `mode` is one of LED_FLASH_MODES ("flash", "rainbow", "cycle"). For the
    animated effects (rainbow/cycle) the drone runs its own palette and the
    r/g/b colour is ignored.
    """
    mode_val = LED_FLASH_MODES.get(mode, LED_FLASH_MODES["flash"])
    if mode == "flash":
        log(f"  LED flash ({r},{g},{b}) mode={mode_val} for {seconds}s")
    else:
        log(f"  LED {mode} effect mode={mode_val} for {seconds}s (colour ignored)")
    server.single_fly_lamplight(r, g, b, int(seconds), mode_val)
    # The drone runs the effect for `time` seconds; wait it out.
    time.sleep(seconds)


# --------------------------------------------------------------------------- #
# Mission thread
# --------------------------------------------------------------------------- #
def run_mission(server, adet, frames, key, lock, state, opts, stop_event, log):
    """Full takeoff -> search -> center -> flash -> retrace -> land sequence."""
    motion = MotionLog(server, led=0, log=log)
    airborne = False

    def frame_size():
        with lock:
            fr = frames.get(key)
        if fr is None:
            return tuple(opts["cell"])
        return fr.width, fr.height

    def current_frame_number():
        with lock:
            fr = frames.get(key)
        return fr.frame_number if fr is not None else -1

    try:
        state["phase"] = "takeoff"
        log("[1] takeoff")
        server.single_fly_takeoff(0, height=opts["height"])
        airborne = True
        time.sleep(1.0)

        state["phase"] = "climb"
        log(f"[1b] climb to {opts['height']} cm (ToF)")
        climb_to_height(server, motion, opts["height"], opts["climb_tol"],
                        opts["climb_step"], opts["settle"], log, stop_event)
        if stop_event.is_set():
            raise _Aborted()

        state["phase"] = f"search:{opts['target']}"
        log(f"[2] search for '{opts['target']}' "
            f"(turnright {opts['search_step']} deg steps)")
        found = search_for_target(
            motion, adet, opts["target"], opts["search_step"], opts["settle"],
            opts["fresh_timeout"], current_frame_number, log, stop_event)
        if stop_event.is_set():
            raise _Aborted()

        if found:
            state["phase"] = "center"
            log("[3] center on target")
            centered = center_on_target(
                motion, adet, opts["target"], frame_size(),
                opts["center_step"], opts["center_min_step"],
                opts["center_deadband"], opts["settle"], opts["fresh_timeout"],
                current_frame_number, opts["center_max_steps"],
                opts["center_retries"], log, stop_event)
            if stop_event.is_set():
                raise _Aborted()

            if centered:
                state["phase"] = "flash"
                log("[4] LED flash")
                flash_led(server, *opts["led_rgb"], opts["flash_seconds"],
                          opts["led_mode"], log)
        else:
            log("  target not found after a full rotation")

        state["phase"] = "return"
        log("[5] return home: retrace %d move(s) in reverse" % len(motion.moves))
        motion.retrace(stop=stop_event)
        log("  touchdown")
        server.single_fly_touchdown(0)
        airborne = False
        state["phase"] = "landed"

    except _Aborted:
        log("mission aborted - landing in place")
    except Exception:  # noqa: BLE001
        log("mission error:\n" + traceback.format_exc())
    finally:
        if airborne:
            try:
                server.single_fly_touchdown(0)
            except Exception:  # noqa: BLE001
                pass
        state["phase"] = "done"
        stop_event.set()


class _Aborted(Exception):
    """Raised internally when the operator asks to stop mid-mission."""


# --------------------------------------------------------------------------- #
# Video pipeline + display
# --------------------------------------------------------------------------- #
def start_stream(drone, key, detector, frames, lock, log):
    from pyhulax.video import AsyncDetector, DrawDetections

    drone.set_video_stream(True)
    stream = drone.create_video_stream()
    adet = AsyncDetector(detector)

    def _capture(frame):
        try:
            with lock:
                frames[key] = frame
        except Exception:  # noqa: BLE001
            pass
        return frame

    stream.add_callback(adet)              # off-thread detection (non-blocking)
    stream.add_callback(DrawDetections())  # draw the latest boxes
    stream.add_callback(_capture)          # stash annotated frame for display
    stream.start()
    log(f"[{key}] detection stream started")
    return stream, adet


def display_loop(key, frames, adet, lock, state, target, cell, stop_event):
    try:
        import cv2
    except ImportError:
        # Headless: log the phase + detections until the mission ends.
        while not stop_event.is_set():
            det = _pick_target(adet.latest_detections, target)
            print(f"[{key}] phase={state.get('phase')} "
                  f"target={'yes' if det else 'no'}", flush=True)
            time.sleep(1.0)
        return

    win = f"Drone {key} - detection flight"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, cell[0], cell[1])
    while not stop_event.is_set():
        with lock:
            fr = frames.get(key)
        if fr is not None:
            img = fr.image
            h, w = img.shape[:2]
            # Frame-center crosshair (the centering target).
            cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255),
                           cv2.MARKER_CROSS, 24, 2)
            det = _pick_target(adet.latest_detections, target)
            if det is not None:
                cx, cy = det.bbox.center
                cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
                cv2.line(img, (w // 2, h // 2), (cx, cy), (0, 0, 255), 1)
            cv2.putText(img, f"phase: {state.get('phase', '-')}",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f"target: {target}   det {adet.avg_inference_time:.0f} ms",
                        (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(win, img)
        if cv2.waitKey(15) & 0xFF == ord("q"):
            stop_event.set()
            break
    try:
        cv2.destroyAllWindows()
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def run(opts):
    log = lambda m: print(m, flush=True)  # noqa: E731
    print(f"=== detection flight: ip={opts['ip']} id={opts['id']} "
          f"target={opts['target']} model={opts['model']} ===")

    detector = _make_detector(opts["model"], opts["confidence"], opts["classes"],
                              opts["imgsz"], log)

    drone = DroneAPI(drone_id=opts["id"])
    try:
        log(f"connecting to {opts['ip']} ...")
        drone.connect(opts["ip"], timeout=opts["connect_timeout"])
    except DroneConnectionError as exc:
        raise SystemExit(f"CONNECT FAILED: {exc}")

    frames: dict = {}
    lock = threading.Lock()
    state: dict = {"phase": "init"}
    stop_event = threading.Event()
    key = f"D{opts['id']}"

    try:
        stream, adet = start_stream(drone, key, detector, frames, lock, log)
    except Exception as exc:  # noqa: BLE001
        drone.disconnect()
        raise SystemExit(f"could not start detection stream: {exc}")

    # Wait for the first frame so centering has real dimensions.
    log("waiting for first video frame ...")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        with lock:
            if frames.get(key) is not None:
                break
        time.sleep(0.1)

    mission = threading.Thread(
        target=run_mission,
        args=(drone._server, adet, frames, key, lock, state, opts, stop_event, log),
        daemon=True,
    )
    mission.start()

    try:
        display_loop(key, frames, adet, lock, state, opts["target"],
                     tuple(opts["cell"]), stop_event)
    except KeyboardInterrupt:
        print("\nInterrupted - aborting mission.")
        stop_event.set()
    finally:
        stop_event.set()
        mission.join(timeout=30.0)
        try:
            adet.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            stream.stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            drone.disconnect()
        except Exception:  # noqa: BLE001
            pass
    print("=== detection flight demo complete ===")


def check(opts):
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    print(f"ip={opts['ip']} id={opts['id']} rtp:{9000 + opts['id'] * 2}")
    print(f"model={opts['model']} target={opts['target']} "
          f"confidence={opts['confidence']} classes={opts['classes'] or 'all'}")
    print("plan:")
    print(f"  1. single_fly_takeoff -> climb to {opts['height']} cm via get_plane_distance")
    print(f"  2. search: single_fly_turnright({opts['search_step']}) steps until '{opts['target']}'")
    print(f"  3. center: strafe until box center within {opts['center_deadband']:.0%} of frame")
    _led_mode_val = LED_FLASH_MODES[opts["led_mode"]]
    print(f"  4. single_fly_lamplight(*{opts['led_rgb']}, {opts['flash_seconds']}, "
          f"{_led_mode_val})  [{opts['led_mode']}]")
    print("  5. retrace recorded moves in reverse, then single_fly_touchdown")

    # Self-test the retrace/inverse logic with a fake server (no hardware).
    class _FakeServer:
        def __init__(self):
            self.calls = []

        def __getattr__(self, name):
            def _rec(*a, **k):
                self.calls.append((name, a[0] if a else None))
            return _rec

    fake = _FakeServer()
    m = MotionLog(fake, led=0, log=lambda *_: None)
    m.up(50); m.right(30); m.turn_right(90); m.down(10)  # noqa: E702
    planned = m.plan_retrace()
    expected = [("up", 10), ("turnleft", 90), ("left", 30), ("down", 50)]
    print(f"  retrace of [up50, right30, turnright90, down10] -> {planned}")
    assert planned == expected, f"retrace mismatch: {planned} != {expected}"

    try:
        import ultralytics  # noqa: F401
        print("ultralytics: available")
    except ImportError:
        print("ultralytics: NOT installed (pip install ultralytics)")
    print("=== check passed ===")


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", metavar="IP", help="Drone IP")
    p.add_argument("--id", type=int, default=1, help="Drone id (default 1)")
    p.add_argument("--target", default="tank",
                   help="Target class label to find (default 'tank'; 'any' = "
                        "largest detection of any class)")
    p.add_argument("--model", default="yolov8n.pt",
                   help="YOLO model path/name (default yolov8n.pt; stock COCO has "
                        "no 'tank' - use a custom model or --target person)")
    p.add_argument("--confidence", type=float, default=0.25,
                   help="Detection confidence threshold 0-1 (default 0.25)")
    p.add_argument("--classes", nargs="+", type=int, default=None, metavar="ID",
                   help="Restrict YOLO to these class ids (must include the target)")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO image size (default 640)")

    p.add_argument("--height", type=int, default=100,
                   help="Target ToF hover height in cm (default 100)")
    p.add_argument("--climb-tol", type=int, default=10,
                   help="ToF height tolerance in cm (default 10)")
    p.add_argument("--climb-step", type=int, default=30,
                   help="Max single up/down nudge while climbing, cm (default 30)")

    p.add_argument("--search-step", type=int, default=30,
                   help="Yaw step per search turn, degrees (default 30)")
    p.add_argument("--center-step", type=int, default=20,
                   help="Max strafe per centering move, cm (default 20). Moves are "
                        "sized adaptively from a learned pixels-per-cm gain and "
                        "capped here to avoid overshooting the target out of frame.")
    p.add_argument("--center-min-step", type=int, default=6,
                   help="Min strafe per centering move, cm (default 6)")
    p.add_argument("--center-deadband", type=float, default=0.08,
                   help="Centered when box center within this fraction of the frame "
                        "(default 0.08)")
    p.add_argument("--center-max-steps", type=int, default=25,
                   help="Max centering iterations (default 25; one axis per iter)")
    p.add_argument("--center-retries", type=int, default=3,
                   help="Re-observations through a transient detection dropout before "
                        "treating the target as lost (default 3)")
    p.add_argument("--settle", type=float, default=1.0,
                   help="Seconds to wait after each move for the video buffer to "
                        "flush and the view to stabilize (default 1.0)")
    p.add_argument("--fresh-timeout", type=float, default=2.0,
                   help="Max seconds to wait for the detector to produce a detection "
                        "from a post-move frame before acting (default 2.0)")

    p.add_argument("--flash-seconds", type=float, default=5.0,
                   help="LED effect duration in seconds (default 5)")
    p.add_argument("--led-mode", choices=list(LED_FLASH_MODES), default="flash",
                   help="LED effect when the target is found: 'flash' blinks the "
                        "--led-rgb colour (BLINK/32), 'rainbow' runs a seven-colour "
                        "cycle (16), 'cycle' cycles R->G->B (4). Default flash.")
    p.add_argument("--led-rgb", nargs=3, type=int, default=[255, 0, 0],
                   metavar=("R", "G", "B"),
                   help="Flash colour as three 0-255 values, e.g. --led-rgb 0 255 0 "
                        "for green (default 255 0 0 = red). Ignored for "
                        "--led-mode rainbow/cycle, which use the drone's own palette.")

    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="Seconds to wait for the drone's heartbeat (default 15)")
    p.add_argument("--cell", nargs=2, type=int, default=[640, 480], metavar=("W", "H"),
                   help="Window size in px (default 640 480)")
    p.add_argument("--check", action="store_true",
                   help="Print the plan, self-test the retrace logic; no hardware")
    args = p.parse_args(argv)

    opts = {
        "ip": args.ip or "0.0.0.0",
        "id": args.id,
        "target": args.target,
        "model": args.model,
        "confidence": args.confidence,
        "classes": args.classes,
        "imgsz": args.imgsz,
        "height": args.height,
        "climb_tol": args.climb_tol,
        "climb_step": args.climb_step,
        "search_step": args.search_step,
        "center_step": args.center_step,
        "center_min_step": args.center_min_step,
        "center_deadband": args.center_deadband,
        "center_max_steps": args.center_max_steps,
        "center_retries": args.center_retries,
        "settle": args.settle,
        "fresh_timeout": args.fresh_timeout,
        "flash_seconds": args.flash_seconds,
        "led_mode": args.led_mode,
        "led_rgb": tuple(args.led_rgb),
        "connect_timeout": args.connect_timeout,
        "cell": args.cell,
    }

    if args.check:
        check(opts)
        return

    if args.ip is None:
        p.error("--ip is required unless using --check")

    run(opts)


if __name__ == "__main__":
    main()
