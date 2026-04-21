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
