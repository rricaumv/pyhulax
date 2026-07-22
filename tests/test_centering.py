"""Tests for the detection-flight centering strategy (skipped without numpy).

Drives examples/object_detection_flight_demo.py::center_on_target against a
simulated target in image space to verify it converges without losing the
target and that dropout retries matter.
"""

import importlib.util
import os
import threading

import pytest

np = pytest.importorskip("numpy")

from pyhulax.video.types import BoundingBox, Detection  # noqa: E402

_DEMO_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "examples", "object_detection_flight_demo.py",
)


def _load_demo():
    spec = importlib.util.spec_from_file_location("odf_demo", _DEMO_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


demo = _load_demo()


class _Sim:
    """A virtual target in a WxH image. Strafing moves the camera, so the box's
    pixel center shifts by (gain px/cm) opposite the strafe intent: strafing
    right pulls a right-of-center target back toward center.
    """

    def __init__(self, cx, cy, gx=3.0, gy=3.0):
        self.cx, self.cy = float(cx), float(cy)
        self.gx, self.gy = gx, gy
        self.min_cx = self.max_cx = self.cx
        self.min_cy = self.max_cy = self.cy

    def _track(self):
        self.min_cx, self.max_cx = min(self.min_cx, self.cx), max(self.max_cx, self.cx)
        self.min_cy, self.max_cy = min(self.min_cy, self.cy), max(self.max_cy, self.cy)

    def right(self, cm): self.cx -= self.gx * cm; self._track()   # noqa: E704
    def left(self, cm):  self.cx += self.gx * cm; self._track()   # noqa: E704
    def down(self, cm):  self.cy -= self.gy * cm; self._track()   # noqa: E704
    def up(self, cm):    self.cy += self.gy * cm; self._track()   # noqa: E704

    def detection(self, label):
        bw = bh = 40
        x = int(round(self.cx)) - bw // 2
        y = int(round(self.cy)) - bh // 2
        # class_id left None so Detection.__post_init__ doesn't derive a colour
        # via cv2 (not installed in the test env); matching is by label anyway.
        return Detection(label=label, confidence=0.9,
                         bbox=BoundingBox(x, y, bw, bh))


class _FakeMotion:
    def __init__(self, sim):
        self.sim = sim
        self.moves = []

    def right(self, cm): self.moves.append(("right", cm)); self.sim.right(cm)  # noqa: E704
    def left(self, cm):  self.moves.append(("left", cm));  self.sim.left(cm)   # noqa: E704
    def up(self, cm):    self.moves.append(("up", cm));    self.sim.up(cm)     # noqa: E704
    def down(self, cm):  self.moves.append(("down", cm));  self.sim.down(cm)   # noqa: E704


class _FakeAdet:
    def __init__(self, sim, label="tank"):
        self.sim = sim
        self.label = label
        self.drop_budget = 0  # number of upcoming observations that return nothing

    def _dets(self):
        if self.drop_budget > 0:
            self.drop_budget -= 1
            return []
        return [self.sim.detection(self.label)]

    @property
    def latest_detections(self):
        return self._dets()

    def wait_for_fresh_detection(self, after_frame_number, timeout=2.0):
        return self._dets()


def _run_center(sim, adet, *, retries=3, max_steps=40):
    motion = _FakeMotion(sim)
    ok = demo.center_on_target(
        motion, adet, "tank", (640, 480),
        20, 6, 10, 0.08, 0.0, 0.1, lambda: 0, max_steps, retries,
        lambda *_: None, threading.Event(),
    )
    return ok, motion


def test_centering_converges_without_losing_target():
    sim = _Sim(cx=560, cy=320, gx=3.0, gy=2.5)  # far off-center, unknown gains
    adet = _FakeAdet(sim)
    ok, _ = _run_center(sim, adet)
    assert ok
    assert abs(sim.cx - 320) <= 0.08 * 640
    assert abs(sim.cy - 240) <= 0.08 * 480
    # The target must never have left the frame while centering.
    assert 0 <= sim.min_cx and sim.max_cx <= 640
    assert 0 <= sim.min_cy and sim.max_cy <= 480


def test_centering_survives_a_transient_dropout():
    sim = _Sim(cx=430, cy=240)          # modest horizontal error
    adet = _FakeAdet(sim)
    adet.drop_budget = 1                # first observation returns nothing
    ok, _ = _run_center(sim, adet, retries=3)
    assert ok                           # retries re-observe and recover


def test_zero_retries_aborts_on_a_single_dropout():
    # Documents why retries matter: with none, one missed frame ends centering.
    sim = _Sim(cx=430, cy=240)
    adet = _FakeAdet(sim)
    adet.drop_budget = 1
    ok, _ = _run_center(sim, adet, retries=0)
    assert ok is False
