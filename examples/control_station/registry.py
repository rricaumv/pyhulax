"""Demo registry for the control station.

Adding a demo to the UI is just building a ``DemoSpec`` and calling
``register(...)``. A ``DemoSpec`` declares:

* the display name + description,
* the optional arguments (``ArgSpec`` list) - the UI auto-builds a form from
  these and hands the collected values back to the runner as an ``opts`` dict,
* a ``runner_factory`` that returns a ``Runner`` (see ``runner.py``). For this
  shell every demo uses ``StubRunner`` (a hardware-free simulation); wiring a
  demo for real flight later means returning a real ``Runner`` instead.

This module is intentionally free of PySide6 and pyhulax imports so it can be
imported and tested headlessly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional

from runner import Runner, StubRunner  # type: ignore  # sibling module


# --------------------------------------------------------------------------- #
# Argument + demo specs
# --------------------------------------------------------------------------- #
@dataclass
class ArgSpec:
    """One optional argument the UI should offer for a demo.

    ``kind`` selects the widget the form builds:
      int | float | str | bool | choice | int_list | str_list
    ``key`` is the dict key passed to the runner; ``flag`` is the CLI-style
    label shown in the form.
    """
    key: str
    flag: str
    kind: str
    default: Any = None
    help: str = ""
    choices: Optional[List[str]] = None
    group: str = "General"
    minimum: float = 0.0
    maximum: float = 10000.0


@dataclass
class DemoSpec:
    key: str
    name: str
    description: str
    args: List[ArgSpec]
    runner_factory: Callable[[], Runner]
    phases: List[str] = field(default_factory=list)


DEMOS: "dict[str, DemoSpec]" = {}


def register(spec: DemoSpec) -> DemoSpec:
    DEMOS[spec.key] = spec
    return spec


def defaults(spec: DemoSpec) -> "dict[str, Any]":
    """The opts dict of an unmodified form (every arg at its default)."""
    return {a.key: a.default for a in spec.args}


# --------------------------------------------------------------------------- #
# Shared argument groups
# --------------------------------------------------------------------------- #
def _connection_args() -> List[ArgSpec]:
    return [
        ArgSpec("ip", "--ip", "str", "192.168.1.58", "Drone IP address",
                group="Connection"),
        ArgSpec("id", "--id", "int", 1, "Drone id", group="Connection",
                minimum=0, maximum=15),
        ArgSpec("connect_timeout", "--connect-timeout", "float", 15.0,
                "Seconds to wait for the drone heartbeat", group="Connection",
                minimum=1, maximum=120),
    ]


def _led_args() -> List[ArgSpec]:
    return [
        ArgSpec("led_mode", "--led-mode", "choice", "rainbow",
                "LED effect", choices=["rainbow", "flash", "cycle"], group="LED"),
        ArgSpec("flash_seconds", "--flash-seconds", "float", 5.0,
                "LED signal duration (s)", group="LED", minimum=0, maximum=60),
    ]


# --------------------------------------------------------------------------- #
# Built-in demo registrations (StubRunner for now)
# --------------------------------------------------------------------------- #
_MINI_TANK_PHASES = ["takeoff", "tilt", "search", "center", "approach",
                     "descend", "led", "return", "landed"]

register(DemoSpec(
    key="mini_tank_approach",
    name="Mini-tank approach",
    description=("Find a 1/72 or 1/35 model tank with a downward-tilted camera, "
                 "center it, approach to a stand-off distance, signal, and retrace "
                 "home."),
    phases=_MINI_TANK_PHASES,
    runner_factory=lambda: StubRunner(_MINI_TANK_PHASES),
    args=_connection_args() + [
        ArgSpec("target", "--target", "str", "tank", "Target class label",
                group="Detection"),
        ArgSpec("model", "--model", "str", "examples/models/tank21jul.pt",
                "YOLO model path", group="Detection"),
        ArgSpec("scale", "--scale", "choice", "1/35", "Model tank scale",
                choices=["1/72", "1/35"], group="Geometry"),
        ArgSpec("hfov", "--hfov", "float", 70.0, "Camera horizontal FOV (deg)",
                group="Geometry", minimum=20, maximum=160),
        ArgSpec("tilt_deg", "--tilt-deg", "float", 45.0,
                "Camera downward tilt (deg)", group="Geometry", minimum=0, maximum=90),
        ArgSpec("height", "--height", "int", 100, "Takeoff/approach height (cm)",
                group="Flight", minimum=30, maximum=300),
        ArgSpec("search_step", "--search-step", "int", 15,
                "Clockwise yaw step (deg)", group="Flight", minimum=1, maximum=90),
        ArgSpec("approach_distance", "--approach-distance", "float", 30.0,
                "Stand-off distance (cm)", group="Flight", minimum=5, maximum=300),
        ArgSpec("descend_cm", "--descend-cm", "int", 50,
                "Descent after reaching the tank (cm)", group="Flight",
                minimum=0, maximum=200),
    ] + _led_args(),
))

_FLIGHT_PHASES = ["takeoff", "climb", "search", "center", "flash", "return", "landed"]

register(DemoSpec(
    key="object_detection_flight",
    name="Detection flight (find + center + flash)",
    description=("Take off, search by yaw until a target class is detected, center "
                 "the box on the frame, flash the LED, then retrace home and land."),
    phases=_FLIGHT_PHASES,
    runner_factory=lambda: StubRunner(_FLIGHT_PHASES),
    args=_connection_args() + [
        ArgSpec("target", "--target", "str", "tank", "Target class label",
                group="Detection"),
        ArgSpec("model", "--model", "str", "yolov8n.pt", "YOLO model path",
                group="Detection"),
        ArgSpec("confidence", "--confidence", "float", 0.25,
                "Detection confidence 0-1", group="Detection", minimum=0, maximum=1),
        ArgSpec("height", "--height", "int", 100, "Takeoff height (cm)",
                group="Flight", minimum=30, maximum=300),
        ArgSpec("search_step", "--search-step", "int", 15, "Yaw search step (deg)",
                group="Flight", minimum=1, maximum=90),
        ArgSpec("center_deadband", "--center-deadband", "float", 0.08,
                "Centered within this fraction of the frame", group="Flight",
                minimum=0.01, maximum=0.5),
    ] + _led_args(),
))

_HOVER_PHASES = ["takeoff", "hover", "land", "landed"]

register(DemoSpec(
    key="takeoff_hover_land",
    name="Takeoff / hover / land",
    description="Simple takeoff to a height, hover for a duration, then land.",
    phases=_HOVER_PHASES,
    runner_factory=lambda: StubRunner(_HOVER_PHASES),
    args=_connection_args() + [
        ArgSpec("height", "--height", "int", 100, "Hover height (cm)",
                group="Flight", minimum=30, maximum=300),
        ArgSpec("hover", "--hover", "float", 5.0, "Hover duration (s)",
                group="Flight", minimum=0, maximum=120),
        ArgSpec("video", "--video", "bool", True, "Show the live video stream",
                group="Flight"),
    ],
))
