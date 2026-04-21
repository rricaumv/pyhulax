# pyhulax

`pyhulax` is a reverse-engineered reconstruction of the original `pyhula`
drone library for the HG-Fly F09-lite / Hula platform.

The original package was narrow in scope, tightly coupled to older code paths,
had major API and design limitations, and was effectively constrained to
Python 3.6 on Windows. This project rebuilds that capability into a modern,
typed, packaged Python SDK that can be installed and used like a real library.

## What This Project Achieved

- reconstructed the control and telemetry surface from the underlying protocol
- built a higher-level [`DroneAPI`][pyhulax.DroneAPI] instead of exposing only awkward low-level helpers
- made the package work on modern Python instead of being stuck on Python 3.6
- removed the original Windows-only constraint from the usable SDK surface
- introduced typed enums, Pydantic models, and explicit exceptions
- added runtime configuration through [`DroneConfig`][pyhulax.config.DroneConfig] with packaged defaults
- added optional video, vision, web, and database features into extras
- documented both the high-level SDK and the underlying runtime internals

## What pyhulax Includes

- high-level control through [`DroneAPI`][pyhulax.DroneAPI]
- typed core types and models in [`pyhulax.core`][pyhulax.core.types.Direction]
- closed-loop flight control in [`pyhulax.control`][pyhulax.control.ManualFlightController]
- optional video, detection, recording, and browser streaming in [`pyhulax.video`][pyhulax.video]
- file and database logging in [`pyhulax.logging`][pyhulax.logging]
- packaged runtime internals in [`pyhulax.fylo`](reference/fylo.md), [`pyhulax.system`][pyhulax.system.datacenter.DataCenter], and [`pyhulax.config`][pyhulax.config.DroneConfig]

This documentation site focuses on that SDK surface, not on the excluded maze
solver, CLI utilities, dashboards, or competition-specific helpers.

pyhulax related code is a heavily modified version of the original `pyhula`
library, but the `pyhulax` module is a new, higher-level API built on top of the underlying protocol. The `pyhulax` module is the recommended way to control the drone.

## Start Here

- [Installation](sdk/installation.md)
- [SDK Overview](sdk/README.md)
- [Configuration](sdk/configuration.md)
- [DroneAPI Guide](sdk/pyhulax.md)
- [Types, Models, and Errors](sdk/types_models_errors.md)
- [Video and Logging](sdk/video_logging.md)

## Reference Sections

Use the API reference when you want the exact callable surface and type
signatures for the packaged modules:

- [`pyhulax`](reference/pyhulax.md)
- [`Config Models`](reference/config.md)
- [`Core Types`](reference/core_types.md)
- [`Core Models`](reference/core_models.md)
- [`Controller Types`](reference/control.md)
- [`Fylo Internals`](reference/fylo.md)
- [`Logging`](reference/logging.md)
- [`System Internals`](reference/system.md)
- [`Video API`](reference/video.md)
