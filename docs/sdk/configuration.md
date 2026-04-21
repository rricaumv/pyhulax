# Configuration

The SDK uses [`DroneConfig`][pyhulax.config.DroneConfig] as its runtime settings object.

Default values are loaded from packaged JSON files in `pyhulax/config/`.
User-supplied config only overrides the fields you explicitly provide.

Primary entrypoints:

- [`DroneAPI(config=DroneConfig(...))`][pyhulax.DroneAPI]
- [`drone.connect()`][pyhulax.DroneAPI.connect] uses [`config.network.drone_ip`][pyhulax.config.NetworkConfig.drone_ip] when no IP is passed
- [`drone.start_video_stream()`][pyhulax.DroneAPI.start_video_stream] uses config-derived video and web defaults
- [`drone.create_flight_controller()`][pyhulax.DroneAPI.create_flight_controller] derives controller defaults from the same config

## Main Types

[`DroneConfig`][pyhulax.config.DroneConfig] is composed of nested immutable config models from [`pyhulax.config`][pyhulax.config.DroneConfig]:

- [`NetworkConfig`][pyhulax.config.NetworkConfig]
- [`ProtocolConfig`][pyhulax.config.ProtocolConfig]
- [`DronePhysicsConfig`][pyhulax.config.DronePhysicsConfig]
- [`FlightConfig`][pyhulax.config.FlightConfig]
- [`ControllerConfig`][pyhulax.config.ControllerConfig]
- [`VideoConfig`][pyhulax.config.VideoConfig]
- [`TimeoutConfig`][pyhulax.config.TimeoutConfig]
- [`BatteryConfig`][pyhulax.config.BatteryConfig]
- [`MediaConfig`][pyhulax.config.MediaConfig]

## Default Configuration

### `NetworkConfig`

- `drone_ip="192.168.100.1"`
- `tcp_port=8888`
- `udp_command_port=8085`
- `udp_status_port=8668`
- `udp_optitrack_port=8688`
- `rtp_base_port=9000`
- `web_port=5000`
- `http_port=12346`

### `ProtocolConfig`

- `command_protocol="tcp"`
- `serial_baudrate=921600`
- `mavlink_system_id=1`
- `mavlink_component_id=2`
- `mavlink_component_file_id=1`

### `DronePhysicsConfig`

- `drone_width_cm=18.93`
- `drone_depth_cm=18.46`
- `min_altitude_cm=30.0`
- `max_altitude_cm=200.0`

### `FlightConfig`

- `default_takeoff_height_cm=80`
- `default_flight_height_cm=100`
- `default_speed_cms=100`
- `position_tolerance_cm=5.0`
- `yaw_tolerance_deg=3.0`

### `ControllerConfig`

- `kp_xy=2.0`
- `kd_xy=0.5`
- `kp_z=3.0`
- `kd_z=0.8`
- `kp_yaw=5.0`
- `kd_yaw=1.0`
- `max_horizontal_output=800`
- `max_vertical_output=600`
- `max_yaw_output=500`
- `control_rate_hz=20.0`

### `VideoConfig`

- `timeout_sec=30.0`
- `buffer_size=10`
- `jpeg_quality=80`
- `max_fps=30.0`
- `detection_confidence=0.5`
- `nms_iou_threshold=0.45`

### `TimeoutConfig`

- `command_timeout_sec=4.0`
- `tcp_connect_timeout_sec=5.0`
- `tcp_recv_timeout_sec=1.0`
- `udp_timeout_sec=1.0`
- `fly_to_timeout_sec=30.0`

### `BatteryConfig`

- `warning_threshold=15`
- `critical_threshold=10`
- `min_operational_threshold=20`

### `MediaConfig`

- `base_dir="media"`
- `photo_dir=None`
- `video_dir=None`
- `log_dir=None`

## Constructing a Config

```python
from pyhulax import (
    DroneConfig,
    NetworkConfig,
    ProtocolConfig,
    FlightConfig,
    MediaConfig,
    VideoConfig,
    TimeoutConfig,
    BatteryConfig,
)

config = DroneConfig(
    network=NetworkConfig(
        drone_ip="192.168.100.42",
        tcp_port=8888,
        web_port=5050,
    ),
    protocol=ProtocolConfig(command_protocol="tcp"),
    flight=FlightConfig(
        default_takeoff_height_cm=90,
        position_tolerance_cm=4.0,
    ),
    media=MediaConfig(
        base_dir="captures",
        photo_dir="photos",
    ),
    video=VideoConfig(timeout_sec=20.0, buffer_size=20),
    timeouts=TimeoutConfig(tcp_connect_timeout_sec=8.0),
    battery=BatteryConfig(warning_threshold=20, critical_threshold=12),
)
```

That mirrors the repo’s real scripts: fix the IP once, choose a flight profile, and set a predictable local media layout.

Sparse override example:

```python
config = DroneConfig(
    network=NetworkConfig(drone_ip="192.168.100.42")
)
```

That only changes `network.drone_ip`. The rest of the network values still come
from `pyhulax/config/network.json`.

Practical single-override example:

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig

config = DroneConfig(
    network=NetworkConfig(drone_ip="192.168.100.42")
)

with DroneAPI(config=config) as drone:
    drone.connect()
    print(drone.default_ip)
```

## Using Config with [`DroneAPI`][pyhulax.DroneAPI]

```python
from pyhulax import DroneAPI

drone = DroneAPI(config=config)

print(drone.default_ip)
print(drone.config.network.web_port)

drone.connect()  # uses config.network.drone_ip
```

Configured media directories are used automatically by capture and download helpers:

```python
from pyhulax import DroneAPI, DroneConfig, MediaConfig

config = DroneConfig(
    media=MediaConfig(
        base_dir="captures",
        photo_dir="photos",
        video_dir="videos",
        log_dir="logs",
    )
)

with DroneAPI(config=config) as drone:
    drone.connect()
    print(drone.take_photo())  # captures/photos/<drone filename>
```

Downloaded media uses [`config.media`][pyhulax.config.DroneConfig.media] by default. [`photo_dir`][pyhulax.config.MediaConfig.photo_dir], [`video_dir`][pyhulax.config.MediaConfig.video_dir], and
[`log_dir`][pyhulax.config.MediaConfig.log_dir] are resolved relative to [`base_dir`][pyhulax.config.MediaConfig.base_dir] unless you give absolute paths.

## Overriding at Call Time

You can still override configured defaults per call:

```python
drone.connect("192.168.100.99")
drone.take_photo(save_path="captures/latest.jpg")
```

Using a directory keeps the original drone filename:

```python
drone.take_photo(save_path="captures/photos")
```

That only overrides the connection target for that call. The instance still retains the original resolved `DroneConfig`.

## Controller Defaults

[`drone.create_flight_controller()`][pyhulax.DroneAPI.create_flight_controller] builds its default [`pyhulax.control.ControllerConfig`][pyhulax.control.ControllerConfig] from:

- [`DroneConfig.controller`][pyhulax.config.DroneConfig.controller]
- [`DroneConfig.flight`][pyhulax.config.DroneConfig.flight]
- [`DroneConfig.physics`][pyhulax.config.DroneConfig.physics]
- [`DroneConfig.timeouts`][pyhulax.config.DroneConfig.timeouts]

That means controller gain, tolerance, altitude clamp, and timeout defaults all come from the same runtime config.

## Video Defaults

[`drone.start_video_stream()`][pyhulax.DroneAPI.start_video_stream] and [`drone.create_video_stream()`][pyhulax.DroneAPI.create_video_stream] use:

- [`config.network.drone_ip`][pyhulax.config.NetworkConfig.drone_ip]
- [`config.network.web_port`][pyhulax.config.NetworkConfig.web_port]
- [`config.network.rtp_base_port`][pyhulax.config.NetworkConfig.rtp_base_port]
- [`config.video.timeout_sec`][pyhulax.config.VideoConfig.timeout_sec]
- [`config.video.buffer_size`][pyhulax.config.VideoConfig.buffer_size]

Example:

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig, VideoConfig

config = DroneConfig(
    network=NetworkConfig(web_port=8080),
    video=VideoConfig(timeout_sec=20.0, buffer_size=20),
)

with DroneAPI(config=config) as drone:
    drone.connect()
    stream = drone.start_video_stream(display=False, web_server=True)
    print(f"http://localhost:{drone.config.network.web_port}")
    stream.stop()
```

## Internal Compatibility Field

[`DroneConfig`][pyhulax.config.DroneConfig] still contains a `maze` field for internal compatibility with non-packaged code in the repo.

Treat that as non-SDK surface. You do not need it to use the packaged API.
