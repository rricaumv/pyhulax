"""Headless tests for the control-station registry + simulated runner.

These exercise the Qt-free modules (registry, runner) so the demo plumbing is
covered without a display or PySide6.
"""

import os
import sys

import pytest

np = pytest.importorskip("numpy")

_CS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "examples", "control_station",
)
if _CS_DIR not in sys.path:
    sys.path.insert(0, _CS_DIR)

import registry  # noqa: E402
from runner import StubRunner  # noqa: E402

_VALID_KINDS = {"int", "float", "str", "bool", "choice", "int_list", "str_list"}


class _CollectingHooks:
    def __init__(self, max_frames=6):
        self.frames = []
        self.telemetry = []
        self.flight = []
        self.logs = []
        self._max = max_frames

    def should_stop(self):
        return len(self.frames) >= self._max

    def emit_frame(self, image, detections=None):
        self.frames.append((image, detections))

    def emit_telemetry(self, data):
        self.telemetry.append(data)

    def emit_flight(self, data):
        self.flight.append(data)

    def emit_log(self, message):
        self.logs.append(message)


def test_registry_has_well_formed_demos():
    assert registry.DEMOS, "no demos registered"
    for key, spec in registry.DEMOS.items():
        assert spec.key == key
        assert spec.name and spec.description
        assert spec.args, f"{key} has no args"
        assert callable(spec.runner_factory)
        keys = [a.key for a in spec.args]
        assert len(keys) == len(set(keys)), f"{key} has duplicate arg keys"
        for a in spec.args:
            assert a.kind in _VALID_KINDS, f"{key}.{a.key} bad kind {a.kind}"
            if a.kind == "choice":
                assert a.choices and a.default in a.choices
        # Every demo can be connected to and configured.
        assert "ip" in keys and "id" in keys
    # defaults() returns a full opts dict.
    spec = next(iter(registry.DEMOS.values()))
    d = registry.defaults(spec)
    assert set(d) == {a.key for a in spec.args}


def test_stub_runner_emits_frames_and_telemetry():
    hooks = _CollectingHooks(max_frames=6)
    StubRunner(["takeoff", "hover", "land"], width=320, height=240).run(
        {"ip": "0.0.0.0", "id": 1, "height": 100}, hooks)
    assert len(hooks.frames) >= 6
    img, _ = hooks.frames[0]
    assert img.shape == (240, 320, 3) and img.dtype == np.uint8
    assert hooks.telemetry and {"battery", "yaw", "height_cm"} <= set(hooks.telemetry[0])
    assert hooks.flight and hooks.flight[0]["phase"] in ("takeoff", "hover", "land")
    assert any("SIMULATION" in m for m in hooks.logs)


def test_mini_tank_factory_builds_a_real_runner():
    from runner import Runner  # noqa: E402
    spec = registry.DEMOS["mini_tank_approach"]
    runner_obj = spec.runner_factory()   # imports runner_real (no pyhulax yet)
    assert isinstance(runner_obj, Runner)


def test_opts_to_argv_maps_keys_to_flags():
    import runner_real  # noqa: E402
    argv = runner_real._opts_to_argv({"ip": "1.2.3.4", "tilt_deg": 50, "led_mode": "rainbow"})
    assert argv == ["--ip", "1.2.3.4", "--tilt-deg", "50", "--led-mode", "rainbow"]


def test_opts_to_argv_laser_off_maps_to_no_laser():
    import runner_real  # noqa: E402
    assert runner_real._opts_to_argv({"laser_mode": "off"}) == ["--no-laser"]
    assert runner_real._opts_to_argv({"laser_mode": "burst"}) == ["--laser-mode", "burst"]


def test_stub_runner_emits_detections_in_detect_phase():
    hooks = _CollectingHooks(max_frames=4)
    StubRunner(["search"], width=320, height=240).run({"id": 1}, hooks)
    # In a detection phase every frame carries a tank box inside the image.
    assert all(dets for _, dets in hooks.frames)
    d = hooks.frames[0][1][0]
    assert d["label"] == "tank"
    assert 0 <= d["x"] < 320 and 0 <= d["y"] < 240
