# DroneAPI Reference

[`DroneAPI`][pyhulax.DroneAPI] is the main high-level SDK class.

Import:

```python
from pyhulax import DroneAPI
```

Related config types:

```python
from pyhulax import DroneConfig, MediaConfig, NetworkConfig
```

## Construction

```python
DroneAPI(
    config: DroneConfig | None = None,
    enable_logging: bool = True,
    flight_logger: FlightLogger | None = None,
    battery_threshold: int | None = None,
    media_dir: Path | str | None = None,
    enable_file_logging: bool = True,
    file_log_dir: str = "logs",
    enable_command_logging: bool = True,
    command_log_dir: str = "logs",
)
```

Important properties:

- `is_connected`
- `config`
- `default_ip`

Context manager support:

```python
with DroneAPI() as drone:
    drone.connect()
```

## Connection and Lifecycle

- [`connect(ip: str | None = None, timeout: float = 5.0) -> None`][pyhulax.DroneAPI.connect]
- [`robust_connect(ip: str | None = None, timeout: float = 5.0, verbose: bool = True) -> bool`][pyhulax.DroneAPI.robust_connect]
- [`disconnect() -> None`][pyhulax.DroneAPI.disconnect]

Typical usage:

```python
drone = DroneAPI(config=config)
drone.connect()
```

Connection flow adapted from the calibration and challenge scripts:

```python
from pyhulax import DroneAPI, DroneConfig, NetworkConfig

config = DroneConfig(
    network=NetworkConfig(drone_ip="192.168.100.1")
)

with DroneAPI(config=config) as drone:
    if not drone.robust_connect(verbose=True):
        raise SystemExit("Failed to connect to drone")

    battery = drone.get_battery()
    print(f"Connected. Battery: {battery}%")
```

## Flight and Movement

Basic movement:

- [`move(direction, distance_cm, led=None, blocking=True, speed=VelocityLevel.ZOOM)`][pyhulax.DroneAPI.move]
- [`rotate(angle_degrees, led=None, blocking=True)`][pyhulax.DroneAPI.rotate]
- [`move_to(x, y, z, led=None, blocking=True, speed=VelocityLevel.ZOOM)`][pyhulax.DroneAPI.move_to]
- [`curve_to(x, y, z, velocity, radius_cm=None, led=None, blocking=True)`][pyhulax.DroneAPI.curve_to]
- [`circle(radius_cm, velocity, clockwise=True, led=None, blocking=True)`][pyhulax.DroneAPI.circle]

Core flight lifecycle:

- [`takeoff(height_cm=None, led=None, reset_yaw=False, with_load=False, blocking=True)`][pyhulax.DroneAPI.takeoff]
- [`land(led=None, blocking=True)`][pyhulax.DroneAPI.land]
- [`hover(led=None, blocking=True)`][pyhulax.DroneAPI.hover]
- [`arm()`][pyhulax.DroneAPI.arm]
- [`disarm()`][pyhulax.DroneAPI.disarm]

Typical flight block:

```python
from pyhulax.core import Direction, VelocityLevel
from pyhulax import DroneAPI

with DroneAPI() as drone:
    drone.connect()
    drone.set_barrier_mode(enabled=True)
    drone.set_qr_localization(enabled=True)

    try:
        drone.takeoff(height_cm=100)
        drone.move(Direction.FORWARD, 100, speed=VelocityLevel.ZOOM)
        drone.rotate(90)
        drone.move_to(50, 100, 100)
        drone.hover()
    finally:
        drone.land()
```

Acrobatic and motion helpers:

- [`flip(direction, led=None, blocking=True)`][pyhulax.DroneAPI.flip]
- [`bounce(height_cm=50, cycles=3, led=None, blocking=True)`][pyhulax.DroneAPI.bounce]
- [`spin(rotations=1, led=None, blocking=True)`][pyhulax.DroneAPI.spin]
- [`vertical_circle(radius_cm, blocking=True)`][pyhulax.DroneAPI.vertical_circle]

## State and Telemetry

- [`get_state() -> DroneState`][pyhulax.DroneAPI.get_state]
- [`get_position() -> Vector3`][pyhulax.DroneAPI.get_position]
- [`get_orientation() -> Orientation`][pyhulax.DroneAPI.get_orientation]
- [`get_battery() -> int`][pyhulax.DroneAPI.get_battery]
- [`get_altitude() -> float`][pyhulax.DroneAPI.get_altitude]
- [`get_velocity() -> Vector3`][pyhulax.DroneAPI.get_velocity]
- [`get_acceleration() -> Vector3`][pyhulax.DroneAPI.get_acceleration]
- [`get_obstacles() -> Obstacles`][pyhulax.DroneAPI.get_obstacles]
- [`get_flight_data() -> FlightData`][pyhulax.DroneAPI.get_flight_data]
- [`get_drone_id() -> int | None`][pyhulax.DroneAPI.get_drone_id]

Telemetry snapshot pattern:

```python
flight = drone.get_flight_data()
state = drone.get_state()

print(flight.position)
print(flight.orientation.yaw)
print(state.obstacles)
print(drone.get_drone_id())
```

## Vision, Color, and QR

- [`recognize_target(target) -> AIResult`][pyhulax.DroneAPI.recognize_target]
- [`recognize_qr(mode=..., timeout=...) -> AIResult`][pyhulax.DroneAPI.recognize_qr]
- [`track_qr(timeout=...) -> AIResult`][pyhulax.DroneAPI.track_qr]
- [`detect_qr(timeout=...) -> AIResult`][pyhulax.DroneAPI.detect_qr]
- [`get_color(mode=1) -> ColorResult`][pyhulax.DroneAPI.get_color]

## LED, Sensors, and Payload

- [`set_led(led, blocking=True)`][pyhulax.DroneAPI.set_led]
- [`enable_led(blocking=True)`][pyhulax.DroneAPI.enable_led]
- [`disable_led(blocking=True)`][pyhulax.DroneAPI.disable_led]
- [`cancel_rgb_animation(blocking=True)`][pyhulax.DroneAPI.cancel_rgb_animation]
- [`set_rgb_brightness(brightness, blocking=True)`][pyhulax.DroneAPI.set_rgb_brightness]
- [`set_avoidance_direction(mode, blocking=True)`][pyhulax.DroneAPI.set_avoidance_direction]
- [`set_electromagnet(on)`][pyhulax.DroneAPI.set_electromagnet]
- [`set_clamp(is_open=None, angle=None)`][pyhulax.DroneAPI.set_clamp]
- [`set_barrier_mode(enabled)`][pyhulax.DroneAPI.set_barrier_mode]
- [`fire_laser(count=1, interval=0.2, blocking=True)`][pyhulax.DroneAPI.fire_laser]
- [`is_laser_hit() -> bool`][pyhulax.DroneAPI.is_laser_hit]

## Camera, Video, and Imaging

- [`take_photo(download=True, timeout=5.0) -> Path | None`][pyhulax.DroneAPI.take_photo]
- [`take_photo(download=True, timeout=5.0, save_path=None) -> Path | None`][pyhulax.DroneAPI.take_photo]
- [`set_video(recording: bool)`][pyhulax.DroneAPI.set_video]
- [`set_video_stream(enabled: bool)`][pyhulax.DroneAPI.set_video_stream]
- [`flip_video()`][pyhulax.DroneAPI.flip_video]
- [`set_camera_angle(mode)`][pyhulax.DroneAPI.set_camera_angle]
- [`set_video_resolution(resolution, blocking=True)`][pyhulax.DroneAPI.set_video_resolution]
- [`start_video_stream(display=True, web_server=False, web_port=None) -> VideoStream`][pyhulax.DroneAPI.start_video_stream]
- [`create_video_stream() -> VideoStream`][pyhulax.DroneAPI.create_video_stream]

## Localization and Autonomy

- [`set_qr_localization(enabled: bool)`][pyhulax.DroneAPI.set_qr_localization]
- [`follow_line(color=..., speed=..., blocking=True) -> LineFollowResult`][pyhulax.DroneAPI.follow_line]

## System and Firmware

- [`get_product_id()`][pyhulax.DroneAPI.get_product_id]
- [`get_firmware_version()`][pyhulax.DroneAPI.get_firmware_version]
- [`get_mcu_version()`][pyhulax.DroneAPI.get_mcu_version]
- [`shutdown()`][pyhulax.DroneAPI.shutdown]
- [`reboot()`][pyhulax.DroneAPI.reboot]
- [`get_storage_capacity()`][pyhulax.DroneAPI.get_storage_capacity]
- [`sync_time()`][pyhulax.DroneAPI.sync_time]

## Wi-Fi and Radio

- [`set_wifi_mode(mode, channel_id=0)`][pyhulax.DroneAPI.set_wifi_mode]
- [`set_wifi_band(band_5ghz=False)`][pyhulax.DroneAPI.set_wifi_band]
- [`set_wifi_power(high=True)`][pyhulax.DroneAPI.set_wifi_power]
- [`set_wifi_broadcast(enabled=True)`][pyhulax.DroneAPI.set_wifi_broadcast]
- [`set_wifi_channel(manual=False, channel_id=0)`][pyhulax.DroneAPI.set_wifi_channel]
- [`set_wifi_ap_mode()`][pyhulax.DroneAPI.set_wifi_ap_mode]
- [`set_velocity_level(level, blocking=True)`][pyhulax.DroneAPI.set_velocity_level]
- [`set_yaw_rate_level(level, blocking=True)`][pyhulax.DroneAPI.set_yaw_rate_level]
- [`set_anti_flicker(hz_50=True)`][pyhulax.DroneAPI.set_anti_flicker]

## Safety and Parameters

- [`enable_battery_failsafe()`][pyhulax.DroneAPI.enable_battery_failsafe]
- [`disable_battery_failsafe()`][pyhulax.DroneAPI.disable_battery_failsafe]
- [`set_parameters(fly_mode=None, opt_mode=None, vision_intensity=None, blocking=True)`][pyhulax.DroneAPI.set_parameters]
- [`set_operate_status(status)`][pyhulax.DroneAPI.set_operate_status]
- [`set_land_speed(fast=False)`][pyhulax.DroneAPI.set_land_speed]

## Media Management

Listing:

- [`list_photos(page=0) -> list[MediaFile]`][pyhulax.DroneAPI.list_photos]
- [`list_videos(page=0) -> list[MediaFile]`][pyhulax.DroneAPI.list_videos]
- [`list_logs(page=0) -> list[MediaFile]`][pyhulax.DroneAPI.list_logs]

Download:

- [`download_photo(photo, save_dir=None) -> Path | None`][pyhulax.DroneAPI.download_photo]
- [`download_video(video, save_dir=None) -> Path | None`][pyhulax.DroneAPI.download_video]
- [`download_log(log, save_dir=None) -> Path | None`][pyhulax.DroneAPI.download_log]
- [`download_all_photos(save_dir=None) -> list[Path]`][pyhulax.DroneAPI.download_all_photos]
- [`download_all_videos(save_dir=None) -> list[Path]`][pyhulax.DroneAPI.download_all_videos]
- [`download_all_logs(save_dir=None) -> list[Path]`][pyhulax.DroneAPI.download_all_logs]

Delete:

- [`delete_photo(photo) -> bool`][pyhulax.DroneAPI.delete_photo]
- [`delete_video(video) -> bool`][pyhulax.DroneAPI.delete_video]
- [`delete_log(log) -> bool`][pyhulax.DroneAPI.delete_log]
- [`delete_all_photos() -> tuple[int, int]`][pyhulax.DroneAPI.delete_all_photos]
- [`delete_all_videos() -> tuple[int, int]`][pyhulax.DroneAPI.delete_all_videos]
- [`delete_all_logs() -> tuple[int, int]`][pyhulax.DroneAPI.delete_all_logs]
- [`delete_all_media() -> dict[str, tuple[int, int]]`][pyhulax.DroneAPI.delete_all_media]

URL helpers:

- [`get_photo_url(photo) -> str`][pyhulax.DroneAPI.get_photo_url]
- [`get_video_url(video) -> str`][pyhulax.DroneAPI.get_video_url]

Photo capture respects [`DroneConfig.media`][pyhulax.config.DroneConfig.media] for default storage, and
[`take_photo(save_path=...)`][pyhulax.DroneAPI.take_photo] can override the destination per call. If
`save_path` points to a directory, the drone filename is preserved. If it
points to a file path such as `captures/latest.jpg`, the download is renamed to
that exact path.

Typical photo and stream usage:

```python
from pyhulax.core import VideoResolution

with DroneAPI() as drone:
    drone.connect()
    drone.set_video_resolution(VideoResolution.LOW)

    stream = drone.start_video_stream(display=False, web_server=True)
    try:
        print(drone.take_photo(save_path="captures/latest.jpg"))
        stream.wait()
    finally:
        stream.stop()
```

## Manual Control

- [`send_manual_control(forward=0.0, right=0.0, up=0.0, rotate=0.0, buttons=0) -> bool`][pyhulax.DroneAPI.send_manual_control]
- [`manual_fly(duration_sec, forward=0.0, right=0.0, up=0.0, rotate=0.0, heartbeat_interval=1.0)`][pyhulax.DroneAPI.manual_fly]
- [`send_app_heartbeat(user_mode=1) -> bool`][pyhulax.DroneAPI.send_app_heartbeat]
- [`set_app_mode(mode: int) -> None`][pyhulax.DroneAPI.set_app_mode]
- [`get_app_mode() -> int`][pyhulax.DroneAPI.get_app_mode]
- [`stop_manual_control() -> bool`][pyhulax.DroneAPI.stop_manual_control]

Use [`set_app_mode(1)`][pyhulax.DroneAPI.set_app_mode] before takeoff if you intend to drive the drone with `MANUAL_CONTROL`.

Manual-control startup pattern:

```python
drone.set_app_mode(1)
drone.set_velocity_level(200)
drone.set_qr_localization(enabled=True)
drone.set_barrier_mode(enabled=True)
```

## Closed-Loop Flight Controller

- [`create_flight_controller(config: FlightControllerConfig | None = None) -> ManualFlightController`][pyhulax.DroneAPI.create_flight_controller]

This returns the PD controller from [`pyhulax.control`][pyhulax.control.ManualFlightController].

Example:

```python
ctrl = drone.create_flight_controller()
result = ctrl.fly_to(x=100, y=150, z=120, yaw=90)
print(result.success, result.reason)
```

This is the same control pattern used by the solver scripts, just without the maze layer:

```python
from pyhulax.control import ControllerConfig

controller = drone.create_flight_controller(
    ControllerConfig(kp_xy=2.5, kp_z=3.5)
)

try:
    drone.takeoff(height_cm=100)
    result = controller.fly_to(x=120, y=0, z=100, yaw=0)
    print(result.success, result.error_position_cm)
finally:
    controller.stop()
    drone.land()
```

## Return Types and Errors

Most command-style methods return [`CommandResult`][pyhulax.core.types.CommandResult].

Telemetry and state methods return typed models from [`pyhulax.core.models`][pyhulax.core.models.DroneState].

Common exceptions:

- [`DroneConnectionError`][pyhulax.core.exceptions.DroneConnectionError]
- [`NotReady`][pyhulax.core.exceptions.NotReady]
- [`LowBattery`][pyhulax.core.exceptions.LowBattery]
- [`TelemetryUnavailable`][pyhulax.core.exceptions.TelemetryUnavailable]

## Notes

- [`connect()`][pyhulax.DroneAPI.connect] and [`robust_connect()`][pyhulax.DroneAPI.robust_connect] use [`config.network.drone_ip`][pyhulax.config.NetworkConfig.drone_ip] if no IP is supplied.
- Maze-oriented helpers are intentionally not part of [`DroneAPI`][pyhulax.DroneAPI].
- Optional video and web features live in [`pyhulax.video`][pyhulax.video] and require extras.
