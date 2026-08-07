# pyhulax

Python API for controlling the HG-Fly F09-lite / Hula drone over MAVLink.

This branch packages the drone SDK only:

- `pyhulax` public API package
- `pyhulax.core`, `pyhulax.fylo`, `pyhulax.system`, `pyhulax.control`, `pyhulax.logging`
- optional video, vision, web, and database extras

It does not publish the maze solver, dashboard, CLI, or competition assets.

## Install

Base API only:

```bash
pip install pyhulax
```

With optional extras:

```bash
pip install "pyhulax[video]"
pip install "pyhulax[vision]"
pip install "pyhulax[web]"
pip install "pyhulax[db]"
pip install "pyhulax[all]"
```

## Quick Start

```python
from pyhulax import DroneAPI
from pyhulax.core import Direction

with DroneAPI() as drone:
    drone.connect("192.168.100.1")
    drone.takeoff()
    drone.move(Direction.FORWARD, 100)
    battery = drone.get_battery()
    print(f"Battery: {battery}%")
    drone.land()
```

## Multi-Drone Control

Each `DroneAPI` owns its own connection and identity (its own `Controlserver`,
`TaskController`, MAVLink encoder, and per-drone telemetry), so you can fly
several drones from one host concurrently:

```python
import threading
from pyhulax import DroneAPI

def fly(ip, drone_id):
    drone = DroneAPI(drone_id=drone_id)   # pin identity for multi-drone
    drone.connect(ip)
    drone.takeoff(height_cm=100)
    drone.hover(5)
    drone.land()
    drone.disconnect()

threads = [
    threading.Thread(target=fly, args=("192.168.1.58", 1)),
    threading.Thread(target=fly, args=("192.168.1.59", 2)),
]
for t in threads: t.start()
for t in threads: t.join()
```

### How identity is resolved (the multi-drone takeoff fix)

The drone only accepts flight commands (e.g. takeoff) from the ground-station
identity it expects. On connect the SDK now:

1. **Detects the local interface that routes to the drone**
   (`socket.connect((drone_ip, 1)); getsockname()`), then binds the TCP and UDP
   sockets to that source IP. 
2. **Derives `bind_client` from that interface's last octet** and stamps every
   command/heartbeat with the identity the drone expects
   (`src_system=255, src_component=bind_client`).
3. **Enters control mode after connect** (`PLANE_COMMAND cmd=10`) so the drone
   accepts `FORMATION_CMD` (takeoff/hover/land).

All of this is automatic — nothing to configure for the common case.

## Demo

A self-contained takeoff / hover / land demo for one and two drones lives at
`examples/takeoff_hover_land_demo.py` (it loads the in-repo `pyhulax`, not an
installed copy):

```bash
# Single drone
python examples/takeoff_hover_land_demo.py one --ips 192.168.1.58 --ids 1

# Two drones, flying concurrently
python examples/takeoff_hover_land_demo.py two \
    --ips 192.168.1.58 192.168.1.59 --ids 1 2

# Two drones with live video, each in its own window
python examples/takeoff_hover_land_demo.py two \
    --ips 192.168.1.58 192.168.1.59 --ids 1 2 --video

# Validate wiring without any hardware (no connect/flight)
python examples/takeoff_hover_land_demo.py two --check
```

Useful flags: `--height` (takeoff/hover height in cm), `--hover` (seconds),
`--connect-timeout` (seconds to wait for the drone's heartbeat), `--video`
(stream each drone in its own window; needs `pip install "pyhulax[video]"`).

`examples/swarm_square_demo.py` is a four-drone choreography: the drones start
at the corners of a 60 cm square, take off (enabling QR-mat localization), fan
out along the diagonals to a 100 cm square, fly two edges clockwise to the
diagonally opposite corner, face forward, and land there — synchronized phase by
phase, with optional 2x2 video windows mirroring the field layout. Flags:
`--qr/--no-qr` (QR-mat localization, default on), `--loop-sides` (edges flown in
step 6; default 2, use 4 for a full loop back to the start corner).

```bash
# 4 drones, order: bottom-left bottom-right top-left top-right
python examples/swarm_square_demo.py \
    --ips 192.168.1.58 192.168.1.70 192.168.1.71 192.168.1.72 \
    --ids 1 2 3 4 --video

# Print the planned geometry/wiring without hardware
python examples/swarm_square_demo.py --check
```

`examples/swarm_vertical_weave_demo.py` is a second four-drone routine: the two
rows (front pair / back pair) swap places twice while held ~80 cm apart in
altitude (60 cm vs 140 cm) so their crossing paths never collide, then return to
their start positions and land. Same flags (`--video`, `--qr/--no-qr`,
`--check`); order of `--ips`/`--ids` is drone 0,1,2,3 (BL, BR, TL, TR).

```bash
python examples/swarm_vertical_weave_demo.py \
    --ips 192.168.1.58 192.168.1.70 192.168.1.71 192.168.1.72 \
    --ids 0 1 2 3 --video
```

`examples/object_detection_demo.py` runs YOLO object detection on the live video
of one or more drones (no flight): it streams each camera, detects + draws boxes
per frame, and shows each drone in its own auto-arranged window. Detection runs
off the decode thread via `pyhulax.video.AsyncDetector` (wrap any detector to
keep the stream smooth — slow inference never stalls the video). Needs the video
and YOLO deps: `pip install "pyhulax[video]" ultralytics`.

```bash
# One drone; press 'q' or Ctrl-C to stop
python examples/object_detection_demo.py --ips 192.168.1.58

# Several drones, larger model, only people + cars (COCO ids 0, 2)
python examples/object_detection_demo.py \
    --ips 192.168.1.58 192.168.1.70 --model yolov8s.pt --classes 0 2
```

`examples/object_detection_flight_demo.py` adds flight to the detector: one drone
takes off and climbs to a target ToF height (`get_plane_distance` +
`single_fly_up/down`), rotates in steps (`single_fly_turnright`) until the
detector reports the target class, strafes (`single_fly_left/right/up/down`) to
put the target's box center on the frame center, flashes its LED
(`single_fly_lamplight(..., mode=32)`) for 5 s, then returns home by retracing
every recorded motion in reverse (inverse move, reverse order) and landing
(`single_fly_touchdown`) — no reliance on absolute-coordinate APIs. Search and
centering only act on a detection computed *after* the drone stopped moving
(`AsyncDetector.wait_for_fresh_detection`), so a stale box from mid-rotation
never triggers "found" or throws off centering (tune with `--settle` /
`--fresh-timeout`). Centering corrects one axis at a time, sizes each strafe
from a pixels-per-cm gain it learns on the fly (so it converges without
overshooting the target out of frame), caps vertical moves lower than lateral
ones (`--center-climb-step`, since they change altitude), retries through
transient detection dropouts (`--center-retries`), and backs off a strafe that
loses the target rather than giving up. Stock
YOLO/COCO has no `tank` class, so use a custom model or `--target person` to
rehearse; `--check` prints the plan and self-tests the retrace logic without
hardware. A ready-made tank model ships at `examples/models/tank21jul.pt`
(`--target tank`).

```bash
# Bundled tank model
python examples/object_detection_flight_demo.py \
    --ip 192.168.1.58 --id 1 --model examples/models/tank21jul.pt --target tank

# Rehearse the full flight against a person with a stock model
python examples/object_detection_flight_demo.py --ip 192.168.1.58 --target person

# Rainbow flash on find; or a custom flash colour (three 0-255 values = R G B)
python examples/object_detection_flight_demo.py --ip 192.168.1.58 --led-mode rainbow
python examples/object_detection_flight_demo.py --ip 192.168.1.58 --led-rgb 0 255 0

# Print the plan + verify retrace logic, no hardware
python examples/object_detection_flight_demo.py --check
```

The "found" flash (step 4) is set by `--led-mode`: `flash` (default) blinks the
single `--led-rgb R G B` colour, `rainbow` runs the drone's seven-colour cycle,
and `cycle` cycles red→green→blue. `--led-rgb` takes three 0-255 values (default
`255 0 0` = red) and is ignored for `rainbow`/`cycle`, which use the drone's own
palette.

`examples/mini_tank_approach_demo.py` hunts a **scale-model tank** (1/72 or
1/35) on the ground: it takes off and climbs to 100 cm, tilts the camera down
(`set_camera_angle`, `--tilt-deg`, default 45°), yaws clockwise in 15° steps
until a tank is detected, centres it in the frame, then flies straight in at
constant height — keeping the tank centred in yaw — until it is ~30 cm away
(`--approach-distance`). Horizontal distance is estimated monocularly from the
box's apparent size, the model's real size (`--scale 1/72|1/35` or
`--tank-size-cm`) and the camera field of view (`--hfov`). It then descends
50 cm (`--descend-cm`), flashes the LED for 5 s (rainbow by default —
`--led-mode`/`--led-rgb`/`--flash-seconds`), and finally retraces every recorded
move in reverse to return home and land. Uses the bundled tank model by default.

```bash
# 1/35 tank, camera down 45°, stop 30 cm away, rainbow flash, retrace home
python examples/mini_tank_approach_demo.py --ip 192.168.1.58 --id 1 --scale 1/35

# 1/72 tank, steeper tilt, tighter stand-off, narrower camera FOV
python examples/mini_tank_approach_demo.py --ip 192.168.1.58 --scale 1/72 \
    --tilt-deg 55 --approach-distance 25 --hfov 66

# Print the plan + self-test the distance/retrace math, no hardware
python examples/mini_tank_approach_demo.py --check
```

## Control Station (GUI)

`examples/control_station/` is a PySide6 desktop app to pick a demo, fill in its
optional arguments, and watch it run — live video with YOLO detections, drone
telemetry (battery, ToF height, attitude, velocity, position), flight info
(connection, mission phase, elapsed, target seen), an info panel (model,
resolution, FPS, inference time) and a log console.

The control station and the `examples/` are **not part of the published PyPI
package** (the wheel ships only the `pyhulax/` package), and the demos load the
in-repo `pyhulax`. So run them from a clone of this repo and install the extras
from the checkout — `pip install "pyhulax[gui]"` from PyPI will not pick up the
`gui` extra:

```bash
# from the repo root
pip install -e ".[gui]"                 # PySide6 + numpy
pip install -e ".[video]" ultralytics   # for the live runner: OpenCV, PyAV, YOLO
python examples/control_station/__main__.py
```

(Or install the deps directly: `pip install PySide6 numpy opencv-python av ultralytics`.)

The **Mini-tank approach (live)** demo flies the real drone through the control
station: it drives `examples/mini_tank_approach_demo.py`'s mission and streams
live video, detections, telemetry, mission phase, and logs into the panels
(needs `pip install "pyhulax[video]" ultralytics` and a connected drone). The
other demos currently run on a hardware-free simulation (`StubRunner`) so the
interface can be reviewed without a drone.

It is built to extend — adding a demo (or wiring an existing one for flight) is
just editing `examples/control_station/registry.py`:

- `ArgSpec`s declare the optional arguments; the UI auto-builds the form and
  hands the values back to the runner as an `opts` dict.
- `runner_factory` returns a `Runner`. Point it at a real `Runner` (see
  `MiniTankRunner` in `runner_real.py`) to fly: it sets up `DroneAPI` + video +
  detector and drives the demo's `run_mission`, pushing frames / detections /
  telemetry / phase / logs through the same hooks the simulation uses. The UI
  itself does not change. `StubRunner` keeps a demo in simulation.

Configured defaults:

```python
from pyhulax import DroneAPI, DroneConfig, MediaConfig, NetworkConfig

config = DroneConfig(
    network=NetworkConfig(
        drone_ip="192.168.100.1",
        tcp_port=8888,
        web_port=5000,
    ),
    media=MediaConfig(
        base_dir="media",
        photo_dir="photos",
    ),
)

drone = DroneAPI(config=config)
drone.connect()  # uses config.network.drone_ip by default
```

The SDK’s default settings are loaded from packaged files in
`pyhulax/config/*.json`. Passing `DroneConfig(...)` only overrides the keys
you explicitly set.

Full SDK docs live in https://pyhulax.xenops.ae

## Optional Features

- `video`: RTP decoding, OpenCV display, recording helpers
- `vision`: ONNX-based detection helpers
- `web`: browser streaming helpers
- `db`: async PostgreSQL flight logging

The core API imports cleanly without these extras installed.

## Public API Areas

- connection lifecycle
- takeoff, landing, movement, rotation
- telemetry and obstacle sensing
- manual control
- media listing, download, and delete
- optional video streaming
- optional structured logging

## Local Verification

Run tests from the project venv:

```bash
.venv/bin/python -m pytest -q
```

Build the wheel:

```bash
uv build --wheel
```
