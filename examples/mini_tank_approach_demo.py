#!/usr/bin/env python3
"""Scale-model-tank approach demo: find a 1/72 or 1/35 tank, drive up, flash, return.

A single drone hunts a miniature model tank on the ground with a downward-tilted
camera, approaches to a set stand-off distance, signals, then retraces its path
home. The mission:

  1. takeoff     single_fly_takeoff + climb to a target ToF height, video on
  2. tilt        tilt the camera down from horizontal (set_camera_angle,
                 DOWN_ABSOLUTE) - default 45 deg (--tilt-deg)
  3. search      yaw clockwise (single_fly_turnright) in 15 deg steps until a
                 tank is detected in view
  4. center      strafe/adjust until the tank's box centre sits on the frame
                 centre (adaptive gain, robust to dropouts)
  5. approach    fly straight in at constant height, keeping the tank centred in
                 yaw, until it's ~30 cm away horizontally (--approach-distance).
                 Distance is estimated monocularly from the box's apparent size,
                 the model's real size (--scale / --tank-size-cm) and the camera
                 field of view (--hfov)
  6. descend     drop by 50 cm (--descend-cm)
  7. LED         flash the LED for 5 s - rainbow by default (--led-mode /
                 --led-rgb / --flash-seconds)
  8. return home retrace every recorded motion in reverse (inverse move, reverse
                 order), then land (single_fly_touchdown)

Detection runs off the decode thread via pyhulax.video.AsyncDetector, so the
stream stays smooth. Distance is a rough monocular estimate (apparent size ->
range), so it depends on a sensible --hfov and the model's real size. The bundled
tank model ships at examples/models/tank21jul.pt and is the default.

Requires the video + YOLO deps:  pip install "pyhulax[video]" ultralytics

Usage:

    # 1/35 tank, camera down 45 deg, stop 30 cm away, rainbow flash, retrace home
    python examples/mini_tank_approach_demo.py --ip 192.168.1.58 --id 1 --scale 1/35

    # 1/72 tank, steeper tilt, custom stand-off + FOV
    python examples/mini_tank_approach_demo.py --ip 192.168.1.58 --scale 1/72 \
        --tilt-deg 55 --approach-distance 25 --hfov 66

    # Print the plan + self-test the distance/retrace math, no hardware
    python examples/mini_tank_approach_demo.py --check

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
import math  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402

from pyhulax import DroneAPI  # noqa: E402
from pyhulax.core import CameraPitchMode, LEDMode  # noqa: E402
from pyhulax.core.exceptions import DroneConnectionError  # noqa: E402


# LED effect for the signal flash (step 7). single_fly_lamplight takes a raw mode
# byte; these are the SDK's LEDMode values.
LED_FLASH_MODES = {
    "rainbow": int(LEDMode.SEVEN_COLOR),  # 16 - multi-colour rainbow cycle
    "flash": int(LEDMode.BLINK),          # 32 - blink the single --led-rgb colour
    "cycle": int(LEDMode.RGB_CYCLE),      # 4  - cycle red -> green -> blue
}

# Real-world size (cm) of a generic MBT's longest dimension at each scale
# (~9.0 m real length). Used to turn apparent box size into a distance estimate.
SCALE_SIZES_CM = {"1/72": 12.5, "1/35": 26.0}

# Bundled tank model (ships in the repo), resolved so it works from any cwd.
_BUNDLED_TANK_MODEL = os.path.join(_REPO_ROOT, "examples", "models", "tank21jul.pt")
DEFAULT_MODEL = _BUNDLED_TANK_MODEL if os.path.isfile(_BUNDLED_TANK_MODEL) else "yolov8n.pt"


# --------------------------------------------------------------------------- #
# Motion log: every executed move is recorded so it can be retraced in reverse
# (inverse move, reverse order) to return home without absolute coordinates.
# --------------------------------------------------------------------------- #
class MotionLog:
    _INVERSE = {
        "up": "down", "down": "up",
        "left": "right", "right": "left",
        "forward": "back", "back": "forward",
        "turnright": "turnleft", "turnleft": "turnright",
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
        getattr(self._server, f"single_fly_{name}")(magnitude, self._led)
        self._moves.append((name, magnitude))

    def forward(self, cm): self._do("forward", cm)   # noqa: E704
    def back(self, cm):    self._do("back", cm)       # noqa: E704
    def up(self, cm):      self._do("up", cm)         # noqa: E704
    def down(self, cm):    self._do("down", cm)       # noqa: E704
    def left(self, cm):    self._do("left", cm)       # noqa: E704
    def right(self, cm):   self._do("right", cm)      # noqa: E704
    def turn_right(self, deg): self._do("turnright", deg)  # noqa: E704 (CW)
    def turn_left(self, deg):  self._do("turnleft", deg)   # noqa: E704 (CCW)

    @property
    def moves(self):
        return list(self._moves)

    def plan_retrace(self):
        return [(self._INVERSE[name], mag) for name, mag in reversed(self._moves)]

    def retrace(self, stop=None) -> None:
        for name, magnitude in self.plan_retrace():
            if stop is not None and stop.is_set():
                self._log("retrace aborted (stop requested)")
                return
            getattr(self._server, f"single_fly_{name}")(magnitude, self._led)
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
    """Largest detection matching `target` (case-insensitive; 'any'/'*' = any)."""
    if not dets:
        return None
    want = target.lower()
    candidates = dets if want in ("any", "*") else [d for d in dets if d.label.lower() == want]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.bbox.area)


def _observe(adet, target, settle, fresh_timeout, current_frame_number, stop, retries=0):
    """Find the target using a detection computed *after* the drone settled.

    Settles the view, notes a freshly-decoded frame, waits for the detector to
    catch up past it, then picks the target. `retries` re-observes (without
    moving) through transient dropouts.
    """
    for attempt in range(retries + 1):
        if stop.is_set():
            return None
        if attempt == 0:
            time.sleep(settle)
        baseline = current_frame_number()
        dets = adet.wait_for_fresh_detection(baseline, timeout=fresh_timeout)
        det = _pick_target(dets, target)
        if det is not None:
            return det
    return None


# --------------------------------------------------------------------------- #
# Monocular distance estimate
# --------------------------------------------------------------------------- #
def _focal_px(frame_w: float, hfov_deg: float) -> float:
    """Pinhole focal length in pixels from frame width and horizontal FOV."""
    return frame_w / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))


def _depression_angle_rad(det, frame_h, focal_px, tilt_deg):
    """Line-of-sight depression below horizontal to the detection.

    The camera axis points `tilt` below level (that's the image centre). A target
    below the image centre is seen at a steeper depression, above it a shallower
    one, by the angle its vertical offset subtends. This is what actually matters
    for forward/back moves - the fixed camera tilt only holds when the target is
    dead-centre vertically.
    """
    dy = det.bbox.center[1] - frame_h / 2.0     # >0 => below centre => steeper
    return math.radians(tilt_deg) + math.atan2(dy, focal_px)


def estimate_horizontal_distance_cm(det, frame_w, frame_h, hfov_deg, tank_size_cm,
                                    tilt_deg):
    """Rough horizontal ground distance (cm) to the tank from its apparent size.

    slant = real_size * focal_px / apparent_px  (pinhole, size-from-range)
    horizontal = slant * cos(depression)        (depression = camera tilt + the
                                                 target's vertical offset angle)

    Using the target's actual depression - not just the camera tilt - keeps the
    forward/back distance right as the tank drifts off the image centre while the
    drone approaches at constant height.
    """
    apparent = max(det.bbox.width, det.bbox.height)
    if apparent <= 0:
        return None
    focal_px = _focal_px(frame_w, hfov_deg)
    slant = tank_size_cm * focal_px / apparent
    depression = _depression_angle_rad(det, frame_h, focal_px, tilt_deg)
    return slant * math.cos(depression)


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
        (motion.up if err > 0 else motion.down)(min(step, abs(err)))
        time.sleep(settle)


def search_for_tank(motion, adet, target, step_deg, settle, fresh_timeout,
                    current_frame_number, log, stop):
    """Yaw clockwise in steps until a tank is detected (or a full turn elapses)."""
    if _observe(adet, target, settle, fresh_timeout, current_frame_number, stop):
        log("  tank already in view")
        return True
    turned = 0
    while turned < 360 and not stop.is_set():
        motion.turn_right(step_deg)  # clockwise; blocking, so the drone stops first
        turned += step_deg
        det = _observe(adet, target, settle, fresh_timeout, current_frame_number, stop)
        log(f"  yawed {turned} deg CW -> {'FOUND' if det else 'no tank'}")
        if det is not None:
            return True
    return False


def center_on_target(motion, adet, target, frame_size, max_step, min_step,
                     climb_step, deadband_frac, settle, fresh_timeout,
                     current_frame_number, max_steps, retries, log, stop):
    """Strafe/adjust until the tank's box centre hits the frame centre.

    One axis per iteration (dominant error first); each strafe is sized from a
    pixels-per-cm gain learned on the fly (so it converges without overshooting
    the tank out of frame); vertical moves are capped lower than lateral ones;
    retries ride through transient dropouts; a lost tank is recovered by backing
    off the last strafe. Every move is recorded for the retrace-home step.
    """
    w, h = frame_size
    cxf, cyf = w / 2.0, h / 2.0
    dbx, dby = w * deadband_frac, h * deadband_frac
    gain = {"x": None, "y": None}

    def observe():
        return _observe(adet, target, settle, fresh_timeout,
                        current_frame_number, stop, retries=retries)

    det = observe()
    if det is None:
        log("  lost tank while centering (not visible at start)")
        return False

    for _ in range(max_steps):
        if stop.is_set():
            return False
        cx, cy = det.bbox.center
        ex, ey = cx - cxf, cy - cyf
        log(f"  center err = ({ex:+.0f}, {ey:+.0f}) px  gain={gain}")
        if abs(ex) <= dbx and abs(ey) <= dby:
            log("  centered")
            return True

        axis = "x" if (abs(ex) / (w / 2.0)) >= (abs(ey) / (h / 2.0)) else "y"
        err = ex if axis == "x" else ey
        axis_max = climb_step if axis == "y" else max_step
        lo = min(min_step, axis_max)
        g = gain[axis]
        step = (abs(err) / g) if (g and g > 1e-6) else 0.5 * axis_max
        step = max(lo, min(axis_max, step))

        if axis == "x":
            forward, undo = (motion.right, motion.left) if err > 0 else (motion.left, motion.right)
        else:
            forward, undo = (motion.down, motion.up) if err > 0 else (motion.up, motion.down)
        forward(step)
        moved_cm = round(step)

        new = observe()
        if new is None:
            log("  tank lost after strafe - backing off to recover")
            undo(step)
            gain[axis] = None
            new = observe()
            if new is None:
                log("  lost tank while centering")
                return False
            det = new
            continue

        ncx, ncy = new.bbox.center
        nerr = (ncx - cxf) if axis == "x" else (ncy - cyf)
        delta = abs(err) - abs(nerr)
        if moved_cm > 0 and delta > 2:
            observed = delta / moved_cm
            gain[axis] = observed if gain[axis] is None else 0.5 * gain[axis] + 0.5 * observed
        elif delta < -2:
            gain[axis] = None
        det = new

    log("  centering hit max steps")
    return abs(det.bbox.center[0] - cxf) <= dbx and abs(det.bbox.center[1] - cyf) <= dby


def approach_tank(motion, adet, target, frame_size, hfov, tank_size_cm, tilt_deg,
                  stop_distance, tol, fwd_step, fwd_step_min, yaw_step,
                  yaw_deadband_frac, settle, fresh_timeout, current_frame_number,
                  max_steps, retries, log, stop):
    """Fly straight in at constant height until ~stop_distance cm away.

    Each iteration: re-observe the tank, keep it centred in yaw (small turns),
    estimate the horizontal distance from its apparent size (compensating for the
    camera tilt + the tank's vertical position in the frame), and step forward by
    at most the remaining gap. Height is never changed here.
    """
    w, h = frame_size
    dbx = w * yaw_deadband_frac
    for _ in range(max_steps):
        if stop.is_set():
            return False
        det = _observe(adet, target, settle, fresh_timeout,
                       current_frame_number, stop, retries=retries)
        if det is None:
            log("  lost tank during approach")
            return False
        cx = det.bbox.center[0]
        ex = cx - w / 2.0
        dist = estimate_horizontal_distance_cm(det, w, h, hfov, tank_size_cm, tilt_deg)
        log(f"  approach: est {('%.0f cm' % dist) if dist else '?'}  x-err={ex:+.0f}px")

        # Keep the tank centred in yaw before driving forward ("approach directly").
        if abs(ex) > dbx:
            (motion.turn_right if ex > 0 else motion.turn_left)(yaw_step)
            continue

        if dist is not None and dist <= stop_distance + tol:
            log(f"  reached ~{stop_distance} cm stand-off")
            return True

        remaining = (dist - stop_distance) if dist is not None else fwd_step
        motion.forward(max(fwd_step_min, min(fwd_step, remaining)))
    det = _observe(adet, target, settle, fresh_timeout, current_frame_number, stop)
    if det is None:
        return False
    dist = estimate_horizontal_distance_cm(det, w, h, hfov, tank_size_cm, tilt_deg)
    return dist is not None and dist <= stop_distance + tol


def flash_led(server, r, g, b, seconds, mode, log):
    """single_fly_lamplight for `seconds` using the chosen effect."""
    mode_val = LED_FLASH_MODES.get(mode, LED_FLASH_MODES["rainbow"])
    if mode == "flash":
        log(f"  LED flash ({r},{g},{b}) mode={mode_val} for {seconds}s")
    else:
        log(f"  LED {mode} effect mode={mode_val} for {seconds}s (colour ignored)")
    server.single_fly_lamplight(r, g, b, int(seconds), mode_val)
    time.sleep(seconds)


# --------------------------------------------------------------------------- #
# Mission thread
# --------------------------------------------------------------------------- #
class _Aborted(Exception):
    """Raised internally when the operator asks to stop mid-mission."""


def run_mission(drone, adet, frames, key, lock, state, opts, stop_event, log):
    server = drone._server
    motion = MotionLog(server, led=0, log=log)
    airborne = False

    def frame_size():
        with lock:
            fr = frames.get(key)
        return (fr.width, fr.height) if fr is not None else tuple(opts["cell"])

    def current_frame_number():
        with lock:
            fr = frames.get(key)
        return fr.frame_number if fr is not None else -1

    try:
        state["phase"] = "takeoff"
        log(f"[1] takeoff + climb to {opts['height']} cm (video already on)")
        server.single_fly_takeoff(0, height=opts["height"])
        airborne = True
        time.sleep(1.0)
        climb_to_height(server, motion, opts["height"], opts["climb_tol"],
                        opts["climb_step"], opts["settle"], log, stop_event)
        if stop_event.is_set():
            raise _Aborted()

        state["phase"] = f"tilt:{opts['tilt_deg']:.0f}"
        log(f"[2] tilt camera down {opts['tilt_deg']:.0f} deg from horizontal")
        drone.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, int(opts["tilt_deg"]))
        time.sleep(opts["settle"])

        state["phase"] = "search"
        log(f"[3] search: yaw CW {opts['search_step']} deg steps until a tank")
        found = search_for_tank(motion, adet, opts["target"], opts["search_step"],
                                opts["settle"], opts["fresh_timeout"],
                                current_frame_number, log, stop_event)
        if stop_event.is_set():
            raise _Aborted()

        if found:
            state["phase"] = "center"
            log("[4] center the tank in view")
            centered = center_on_target(
                motion, adet, opts["target"], frame_size(), opts["center_step"],
                opts["center_min_step"], opts["center_climb_step"],
                opts["center_deadband"], opts["settle"], opts["fresh_timeout"],
                current_frame_number, opts["center_max_steps"],
                opts["center_retries"], log, stop_event)
            if stop_event.is_set():
                raise _Aborted()

            reached = False
            if centered:
                state["phase"] = "approach"
                log(f"[5] approach to ~{opts['approach_distance']} cm (constant height)")
                reached = approach_tank(
                    motion, adet, opts["target"], frame_size(), opts["hfov"],
                    opts["tank_size_cm"], opts["tilt_deg"], opts["approach_distance"],
                    opts["approach_tol"], opts["forward_step"], opts["forward_step_min"],
                    opts["approach_yaw_step"], opts["approach_yaw_deadband"],
                    opts["settle"], opts["fresh_timeout"], current_frame_number,
                    opts["approach_max_steps"], opts["center_retries"], log, stop_event)
                if stop_event.is_set():
                    raise _Aborted()

            if reached:
                state["phase"] = "descend"
                log(f"[6] descend {opts['descend_cm']} cm")
                motion.down(opts["descend_cm"])

                state["phase"] = "led"
                log("[7] LED signal")
                flash_led(server, *opts["led_rgb"], opts["flash_seconds"],
                          opts["led_mode"], log)
            else:
                log("  did not reach stand-off distance")
        else:
            log("  no tank found after a full rotation")

        # [8] return home: level the camera, retrace every recorded move, land.
        state["phase"] = "return"
        log("[8] return home: level camera + retrace %d move(s), then land"
            % len(motion.moves))
        try:
            drone.set_camera_angle(CameraPitchMode.UP_ABSOLUTE, 0)
        except Exception:  # noqa: BLE001
            pass
        motion.retrace(stop=stop_event)
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


# --------------------------------------------------------------------------- #
# Video pipeline + display
# --------------------------------------------------------------------------- #
def start_stream(drone, key, detector, frames, lock, state, log):
    from pyhulax.video import AsyncDetector, DrawDetections

    drone.set_video_stream(True)
    stream = drone.create_video_stream()
    adet = AsyncDetector(detector)
    draw = DrawDetections()  # draws the detector's original YOLO boxes

    def _draw_unless_centering(frame):
        # Same box rendering as the other demos, but suppressed during the
        # centering phase so only the crosshair shows.
        if not str(state.get("phase", "")).startswith("center"):
            return draw(frame)
        return frame

    def _capture(frame):
        try:
            with lock:
                frames[key] = frame
        except Exception:  # noqa: BLE001
            pass
        return frame

    stream.add_callback(adet)                  # off-thread detection
    stream.add_callback(_draw_unless_centering)  # original YOLO boxes (not while centering)
    stream.add_callback(_capture)              # stash annotated frame for display
    stream.start()
    log(f"[{key}] detection stream started")
    return stream, adet


def display_loop(key, frames, adet, lock, state, opts, stop_event):
    target = opts["target"]
    try:
        import cv2
    except ImportError:
        while not stop_event.is_set():
            det = _pick_target(adet.latest_detections, target)
            print(f"[{key}] phase={state.get('phase')} tank={'yes' if det else 'no'}",
                  flush=True)
            time.sleep(1.0)
        return

    win = f"Drone {key} - tank approach"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, opts["cell"][0], opts["cell"][1])
    while not stop_event.is_set():
        with lock:
            fr = frames.get(key)
        if fr is not None:
            # The YOLO box is already drawn on the frame by the DrawDetections
            # callback (except during centering). Here we only add overlays.
            img = fr.image
            h, w = img.shape[:2]
            phase = state.get("phase", "")
            centering = phase.startswith("center")

            # Distance label next to the tank (no box; the box is already drawn).
            if not centering:
                tank = _pick_target(fr.detections or [], target)
                if tank is not None:
                    cx, cy = tank.bbox.center
                    dist = estimate_horizontal_distance_cm(
                        tank, w, h, opts["hfov"], opts["tank_size_cm"], opts["tilt_deg"])
                    if dist is not None:
                        cv2.putText(img, f"~{dist:.0f} cm", (cx + 8, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(img, f"phase: {phase or '-'}",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f"tilt {opts['tilt_deg']:.0f}  det {adet.avg_inference_time:.0f} ms",
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
    print(f"=== mini-tank approach: ip={opts['ip']} id={opts['id']} "
          f"scale={opts['scale']} tank~{opts['tank_size_cm']}cm model={opts['model']} ===")

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
        stream, adet = start_stream(drone, key, detector, frames, lock, state, log)
    except Exception as exc:  # noqa: BLE001
        drone.disconnect()
        raise SystemExit(f"could not start detection stream: {exc}")

    log("waiting for first video frame ...")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        with lock:
            if frames.get(key) is not None:
                break
        time.sleep(0.1)

    mission = threading.Thread(
        target=run_mission,
        args=(drone, adet, frames, key, lock, state, opts, stop_event, log),
        daemon=True,
    )
    mission.start()

    try:
        display_loop(key, frames, adet, lock, state, opts, stop_event)
    except KeyboardInterrupt:
        print("\nInterrupted - aborting mission.")
        stop_event.set()
    finally:
        stop_event.set()
        mission.join(timeout=30.0)
        for fn in (adet.stop, stream.stop, drone.disconnect):
            try:
                fn()
            except Exception:  # noqa: BLE001
                pass
    print("=== mini-tank approach demo complete ===")


def check(opts):
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    print(f"ip={opts['ip']} id={opts['id']} rtp:{9000 + opts['id'] * 2}")
    print(f"model={opts['model']} target={opts['target']}")
    print(f"scale={opts['scale']} tank_size={opts['tank_size_cm']}cm hfov={opts['hfov']}")
    print("plan:")
    print(f"  1. single_fly_takeoff -> climb to {opts['height']} cm (video on)")
    print(f"  2. set_camera_angle(DOWN_ABSOLUTE, {opts['tilt_deg']:.0f})")
    print(f"  3. search: single_fly_turnright({opts['search_step']}) CW until 'tank'")
    print(f"  4. center the tank's box on the frame centre")
    print(f"  5. approach to ~{opts['approach_distance']:.0f} cm (constant height)")
    print(f"  6. single_fly_down({opts['descend_cm']})")
    _led = LED_FLASH_MODES[opts["led_mode"]]
    print(f"  7. single_fly_lamplight(*{opts['led_rgb']}, {opts['flash_seconds']}, "
          f"{_led})  [{opts['led_mode']}]")
    print(f"  8. level camera, retrace recorded moves in reverse, single_fly_touchdown")

    # Self-test the distance estimate.
    w, h = opts["cell"][0], opts["cell"][1]

    class _Box:
        def __init__(self, s, cy): self.width = self.height = s; self.center = (w / 2, cy)
    class _Det:
        def __init__(self, s, cy=None): self.bbox = _Box(s, h / 2 if cy is None else cy)

    def _dist(det):
        return estimate_horizontal_distance_cm(det, w, h, opts["hfov"],
                                               opts["tank_size_cm"], opts["tilt_deg"])

    # Closer (bigger box) => smaller distance.
    d_far, d_near = _dist(_Det(50)), _dist(_Det(200))
    print(f"  distance check: box 50px -> {d_far:.0f} cm, 200px -> {d_near:.0f} cm")
    assert d_near < d_far, "closer (bigger box) must estimate nearer"

    # Tilt compensation: same box lower in the frame is at a steeper depression,
    # so its horizontal distance is smaller than if it were dead-centre.
    d_centre, d_low = _dist(_Det(80)), _dist(_Det(80, cy=h * 0.85))
    print(f"  tilt compensation: centred {d_centre:.0f} cm vs low-in-frame {d_low:.0f} cm")
    assert d_low < d_centre, "a lower target must project to a nearer horizontal distance"

    # Self-test the retrace inverse (inverse move, reverse order).
    ml = MotionLog(_FakeServer(), log=lambda *_: None)
    ml.forward(50); ml.turn_right(15); ml.down(20)  # noqa: E702
    planned = ml.plan_retrace()
    expected = [("up", 20), ("turnleft", 15), ("back", 50)]
    print(f"  retrace of [forward50, turnright15, down20] -> {planned}")
    assert planned == expected, f"retrace mismatch: {planned}"
    print("=== check passed ===")


class _FakeServer:
    def __getattr__(self, name):
        def _rec(*a, **k):
            return None
        return _rec


def _resolve_tank_size(args):
    if args.tank_size_cm is not None:
        return args.tank_size_cm
    return SCALE_SIZES_CM[args.scale]


def _build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", metavar="IP", help="Drone IP")
    p.add_argument("--id", type=int, default=1, help="Drone id (default 1)")
    p.add_argument("--target", default="tank",
                   help="Target class label (default 'tank'; 'any' = largest object)")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help="YOLO model path (default: bundled examples/models/tank21jul.pt)")
    p.add_argument("--confidence", type=float, default=0.25,
                   help="Detection confidence threshold 0-1 (default 0.25)")
    p.add_argument("--classes", nargs="+", type=int, default=None, metavar="ID",
                   help="Restrict YOLO to these class ids")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO image size (default 640)")

    # Geometry / distance estimate
    p.add_argument("--scale", choices=list(SCALE_SIZES_CM), default="1/35",
                   help="Model tank scale; sets the assumed real size "
                        f"({', '.join(f'{k}->{v}cm' for k, v in SCALE_SIZES_CM.items())})")
    p.add_argument("--tank-size-cm", type=float, default=None,
                   help="Override the tank's real longest dimension in cm "
                        "(otherwise derived from --scale)")
    p.add_argument("--hfov", type=float, default=70.0,
                   help="Camera horizontal field of view in degrees (default 70)")
    p.add_argument("--tilt-deg", type=float, default=45.0,
                   help="Camera downward tilt from horizontal, 0-90 (default 45)")

    # Flight
    p.add_argument("--height", type=int, default=100,
                   help="Takeoff/approach ToF height in cm (default 100)")
    p.add_argument("--climb-tol", type=int, default=10, help="ToF height tolerance cm")
    p.add_argument("--climb-step", type=int, default=30, help="Max climb nudge cm")
    p.add_argument("--search-step", type=int, default=15,
                   help="Clockwise yaw step per search turn, degrees (default 15)")

    # Centering (step 4)
    p.add_argument("--center-step", type=int, default=20,
                   help="Max lateral strafe per centering move, cm (default 20)")
    p.add_argument("--center-climb-step", type=int, default=10,
                   help="Max vertical move per centering step, cm (default 10)")
    p.add_argument("--center-min-step", type=int, default=6,
                   help="Min strafe per centering move, cm (default 6)")
    p.add_argument("--center-deadband", type=float, default=0.08,
                   help="Centered when box centre within this fraction of the frame")
    p.add_argument("--center-max-steps", type=int, default=25,
                   help="Max centering iterations (default 25)")
    p.add_argument("--center-retries", type=int, default=3,
                   help="Re-observations through a transient dropout (default 3)")

    # Approach (step 5)
    p.add_argument("--approach-distance", type=float, default=30.0,
                   help="Horizontal stand-off distance to stop at, cm (default 30)")
    p.add_argument("--approach-tol", type=float, default=8.0,
                   help="Stand-off distance tolerance, cm (default 8)")
    p.add_argument("--forward-step", type=int, default=20,
                   help="Max forward move per approach step, cm (default 20)")
    p.add_argument("--forward-step-min", type=int, default=8,
                   help="Min forward move per approach step, cm (default 8)")
    p.add_argument("--approach-yaw-step", type=int, default=8,
                   help="Yaw correction per approach step to re-centre, deg (default 8)")
    p.add_argument("--approach-yaw-deadband", type=float, default=0.10,
                   help="Re-centre yaw when the tank is beyond this fraction of the "
                        "frame width from centre (default 0.10)")
    p.add_argument("--approach-max-steps", type=int, default=30,
                   help="Max approach iterations (default 30)")
    p.add_argument("--descend-cm", type=int, default=50,
                   help="Descent after reaching the tank, cm (default 50)")

    # LED signal (pattern / colour / time all optional)
    p.add_argument("--led-mode", choices=list(LED_FLASH_MODES), default="rainbow",
                   help="LED effect: rainbow (default), flash (single --led-rgb), cycle")
    p.add_argument("--led-rgb", nargs=3, type=int, default=[255, 0, 0],
                   metavar=("R", "G", "B"),
                   help="Flash colour for --led-mode flash (default 255 0 0; ignored "
                        "for rainbow/cycle)")
    p.add_argument("--flash-seconds", type=float, default=5.0,
                   help="LED signal duration in seconds (default 5)")

    # Misc
    p.add_argument("--settle", type=float, default=1.0,
                   help="Seconds to wait after each move for the view to settle")
    p.add_argument("--fresh-timeout", type=float, default=2.0,
                   help="Max seconds to wait for a post-move detection (default 2)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="Seconds to wait for the drone's heartbeat (default 15)")
    p.add_argument("--cell", nargs=2, type=int, default=[640, 480], metavar=("W", "H"),
                   help="Window size in px (default 640 480)")
    p.add_argument("--check", action="store_true",
                   help="Print the plan + self-test distance/retrace math; no hardware")
    return p


def build_opts(argv=None):
    """Parse argv into the full opts dict (all defaults + any overrides).

    Reused by the control-station runner so the GUI form only needs to supply a
    subset of arguments; everything else keeps its CLI default.
    """
    p = _build_parser()
    args = p.parse_args(argv)

    opts = {
        "ip": args.ip or "0.0.0.0",
        "id": args.id,
        "target": args.target,
        "model": args.model,
        "confidence": args.confidence,
        "classes": args.classes,
        "imgsz": args.imgsz,
        "scale": args.scale,
        "tank_size_cm": _resolve_tank_size(args),
        "hfov": args.hfov,
        "tilt_deg": max(0.0, min(90.0, args.tilt_deg)),
        "height": args.height,
        "climb_tol": args.climb_tol,
        "climb_step": args.climb_step,
        "search_step": args.search_step,
        "center_step": args.center_step,
        "center_climb_step": args.center_climb_step,
        "center_min_step": args.center_min_step,
        "center_deadband": args.center_deadband,
        "center_max_steps": args.center_max_steps,
        "center_retries": args.center_retries,
        "approach_distance": args.approach_distance,
        "approach_tol": args.approach_tol,
        "forward_step": args.forward_step,
        "forward_step_min": args.forward_step_min,
        "approach_yaw_step": args.approach_yaw_step,
        "approach_yaw_deadband": args.approach_yaw_deadband,
        "approach_max_steps": args.approach_max_steps,
        "descend_cm": args.descend_cm,
        "led_mode": args.led_mode,
        "led_rgb": tuple(args.led_rgb),
        "flash_seconds": args.flash_seconds,
        "settle": args.settle,
        "fresh_timeout": args.fresh_timeout,
        "connect_timeout": args.connect_timeout,
        "cell": args.cell,
    }
    return opts, args, p


def main(argv=None):
    opts, args, p = build_opts(argv)
    if args.check:
        check(opts)
        return
    if args.ip is None:
        p.error("--ip is required unless using --check")
    run(opts)


if __name__ == "__main__":
    main()
