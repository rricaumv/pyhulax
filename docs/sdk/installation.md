# Installation

Install `pyhulax` like a normal Python package.

## Base Package

```bash
pip install pyhulax
```

That gives you the core SDK:

- [`DroneAPI`][pyhulax.DroneAPI]
- [`pyhulax.core`][pyhulax.core.types.Direction]
- [`pyhulax.control`][pyhulax.control.ManualFlightController]
- [`pyhulax.logging`][pyhulax.logging]
- packaged runtime layers in [`pyhulax.config`][pyhulax.config.DroneConfig], [`pyhulax.fylo`](../reference/fylo.md), and [`pyhulax.system`][pyhulax.system.datacenter.DataCenter]

## Optional Extras

Install only the optional layers you need:

```bash
pip install "pyhulax[video]"
pip install "pyhulax[vision]"
pip install "pyhulax[web]"
pip install "pyhulax[db]"
pip install "pyhulax[all]"
```

What they add:

- `video`: RTP/RTSP streaming, display, recording helpers
- `vision`: ONNX- and model-backed detection helpers
- `web`: browser streaming helpers
- `db`: PostgreSQL-backed flight logging
- `all`: installs every optional extra

## Local Development Install

From the repo:

```bash
uv sync
uv run python -c "from pyhulax import DroneAPI; print(DroneAPI)"
```

Editable install with pip-compatible tooling:

```bash
uv pip install -e .
```

## Python Version

`pyhulax` targets modern Python environments. This project exists partly
because the original `pyhula` release was effectively stuck on Python 3.6 and
Windows-only assumptions.

If you want a local Python 3.13 environment with `uv` and `pyenv`:

```bash
pyenv local 3.13.3
uv venv --python 3.13.3 .venv
source .venv/bin/activate
python -V
```

## Verify the Install

Basic import check:

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig
from pyhulax.core import Direction

config = DroneConfig(
    network=NetworkConfig(drone_ip="192.168.100.1")
)

drone = DroneAPI(config=config, enable_command_logging=False)
print(drone.default_ip)
print(Direction.FORWARD)
```

If you installed video extras, you can also verify the streaming layer imports:

```python
from pyhulax.video import VideoStream, Frame, VideoDisplay

print(VideoStream, Frame, VideoDisplay)
```

If you installed database extras:

```python
from pyhulax.logging import SQLiteLogger, PostgresLogger

print(SQLiteLogger, PostgresLogger)
```

## Next Steps

- [SDK Overview](./README.md)
- [Configuration](./configuration.md)
- [DroneAPI Guide](./pyhulax.md)
- [Video and Logging](./video_logging.md)
