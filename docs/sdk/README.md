# pyhulax SDK

`pyhulax` is the reconstructed, modernized successor to the original
`pyhula` package.

The original library exposed a limited and awkward API surface, mixed
application logic with transport code, and was only released for Python 3.6 on
Windows. `pyhulax` rebuilds that stack into a typed Python SDK that can be
configured, packaged, tested, and used on modern Python environments.

This section documents the packaged SDK surface that ships in the wheel:

- [`pyhulax`][pyhulax.DroneAPI]
- [`pyhulax.core`][pyhulax.core.types.Direction]
- [`pyhulax.control`][pyhulax.control.ManualFlightController]
- [`pyhulax.logging`][pyhulax.logging]
- [`pyhulax.video`][pyhulax.video]
- the runtime internals in [`pyhulax.fylo`](../reference/fylo.md), [`pyhulax.system`][pyhulax.system.datacenter.DataCenter], and [`pyhulax.config`][pyhulax.config.DroneConfig]

It does not include the CLI, maze solver, dashboard, or competition
helpers as part of the supported SDK surface.

## Why pyhulax Exists

This project was built to solve the main problems in the original `pyhula`:

- Python 3.6 and Windows-only packaging assumptions
- poor separation between library code and application code
- weak typing and unclear return values
- hardcoded defaults spread across the runtime
- limited documentation and poor discoverability of the actual callable surface

## What Changed

Compared with the original `pyhula`, `pyhulax` now provides:

- a high-level [`DroneAPI`][pyhulax.DroneAPI] for connection, movement, telemetry, media, and device control
- typed config models via [`DroneConfig`][pyhulax.config.DroneConfig]
- typed enums, results, and state models in [`pyhulax.core`][pyhulax.core.types.Direction]
- an optional closed-loop PD controller in [`pyhulax.control`][pyhulax.control.ManualFlightController]
- optional video, detection, web, and logging layers split into extras
- packaged defaults loaded from the shipped config files instead of scattered literals

## What This Docs Section Covers

This section is about using `pyhulax` as a Python SDK:

- how to configure it
- how to connect and fly
- what types and return models it uses
- how to work with video, media, and logging
- where the lower-level runtime layers live when you need to inspect them

## SDK Map

- [Installation](./installation.md)
- [Getting Started](./README.md)
- [Configuration](./configuration.md)
- [DroneAPI Reference](./pyhulax.md)
- [Types, Models, and Errors](./types_models_errors.md)
- [Video and Logging](./video_logging.md)
- [API Reference](../reference/pyhulax.md)

## Package Contents

Public package entrypoints:

- [`pyhulax`][pyhulax.DroneAPI]
- [`pyhulax.core`][pyhulax.core.types.Direction]
- [`pyhulax.control`][pyhulax.control.ManualFlightController]
- [`pyhulax.logging`][pyhulax.logging]
- [`pyhulax.video`][pyhulax.video]

Packaged runtime modules:

- [`pyhulax.config`][pyhulax.config.DroneConfig]
- [`pyhulax.fylo`](../reference/fylo.md)
- [`pyhulax.system`][pyhulax.system.datacenter.DataCenter]

Excluded from the release artifact:

- `cli/`
- `pyhulax/maze/`
- `pyhulax/system/dancecontroller.py`
- `pyhulax/system/dancefileanalyzer.py`

## Practical Result

The outcome is that `pyhulax` can now be used like a normal modern SDK:

- install it from a package instead of copying legacy source around
- initialize it with defaults or a typed config object
- call a coherent high-level API instead of stitching together low-level pieces
- use telemetry, controller, media, streaming, and logging support from one package
- keep optional dependencies optional

For actual install commands and environment setup, use the dedicated [Installation](./installation.md) page.

## Quick Start

```python
from pyhulax import DroneAPI
from pyhulax.core import Direction

with DroneAPI() as drone:
    drone.connect()
    drone.takeoff()
    drone.move(Direction.FORWARD, 100)
    print(drone.get_battery())
    drone.land()
```

## Typical Session Pattern

Most real scripts in this repo follow the same shape:

1. construct [`DroneAPI`][pyhulax.DroneAPI]
2. connect and check battery
3. enable the modes you need
4. take off, do the work, always land in `finally`

```python
from pyhulax import DroneAPI

with DroneAPI() as drone:
    if not drone.robust_connect(verbose=True):
        raise SystemExit("Check Wi-Fi connection to the drone")

    battery = drone.get_battery()
    print(f"Battery: {battery}%")

    if battery < 20:
        raise SystemExit("Battery too low for flight")

    drone.set_barrier_mode(enabled=True)
    drone.set_qr_localization(enabled=True)

    try:
        drone.takeoff()
        print(drone.get_position())
        print(drone.get_state())
    finally:
        drone.land()
```

## Typed Configuration

[`DroneAPI`][pyhulax.DroneAPI] accepts a typed runtime config object. Defaults still exist, but they now live in [`DroneConfig`][pyhulax.config.DroneConfig] instead of being scattered as hardcoded literals.

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig

config = DroneConfig(
    network=NetworkConfig(
        drone_ip="192.168.100.1",
        tcp_port=8888,
        web_port=5000,
    )
)

drone = DroneAPI(config=config)
drone.connect()  # uses config.network.drone_ip
```

Media defaults can also be part of the config:

```python
from pyhulax import DroneAPI, DroneConfig, MediaConfig

config = DroneConfig(
    media=MediaConfig(
        base_dir="captures",
        photo_dir="photos",
        video_dir="videos",
    )
)

with DroneAPI(config=config) as drone:
    drone.connect()
    photo = drone.take_photo()  # saves under captures/photos/
    print(photo)
```

## Main SDK Concepts

- [`DroneAPI`][pyhulax.DroneAPI] is the high-level typed control surface.
- [`DroneConfig`][pyhulax.config.DroneConfig] defines network, protocol, timing, controller, video, and battery defaults.
- [`pyhulax.core`][pyhulax.core.types.Direction] contains enums, models, and exceptions used by the API.
- [`ManualFlightController`][pyhulax.control.ManualFlightController] is the closed-loop PD controller for continuous xyz/yaw control.
- [`pyhulax.video`][pyhulax.video] contains optional streaming, display, web, recording, and detection helpers.
- [`pyhulax.logging`][pyhulax.logging] contains file and database logging helpers.

## Auto-Generated Reference

This repo now includes a MkDocs + mkdocstrings reference site generated from
the Python source itself.

Reference sections live under:

- `docs/reference/pyhulax.md`
- `docs/reference/config.md`
- `docs/reference/core_types.md`
- `docs/reference/core_models.md`
- `docs/reference/exceptions.md`
- `docs/reference/control.md`
- `docs/reference/fylo.md`
- `docs/reference/video_types.md`

Run locally with:

```bash
uv run mkdocs serve
```

## Recommended Usage Pattern

1. Construct one [`DroneConfig`][pyhulax.config.DroneConfig] for your environment.
2. Pass that config into [`DroneAPI`][pyhulax.DroneAPI].
3. Call [`connect()`][pyhulax.DroneAPI.connect] without repeating the IP unless you need to override it.
4. Use typed enums and models from [`pyhulax.core`][pyhulax.core.types.Direction].
5. Keep optional dependencies behind extras.

## Example

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig
from pyhulax.core import Direction, VelocityLevel

config = DroneConfig(
    network=NetworkConfig(drone_ip="192.168.100.1"),
)

with DroneAPI(config=config) as drone:
    drone.connect()
    drone.takeoff()
    drone.move(Direction.FORWARD, 150, speed=VelocityLevel.ZOOM)
    state = drone.get_state()
    print(state.position, state.battery_percent)
    drone.land()
```

## Common Workflows

Take a photo to a fixed path:

```python
from pyhulax import DroneAPI

with DroneAPI() as drone:
    drone.connect()
    path = drone.take_photo(save_path="captures/latest.jpg")
    print(path)
```

Start a video stream and keep it open until interrupted:

```python
from pyhulax import DroneAPI

with DroneAPI() as drone:
    drone.connect()
    stream = drone.start_video_stream(display=True, web_server=True)
    print(f"Web stream on http://localhost:{drone.config.network.web_port}")

    try:
        stream.wait()
    finally:
        stream.stop()
```
