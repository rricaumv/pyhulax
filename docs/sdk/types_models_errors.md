# Types, Models, and Errors

The SDK is designed around typed enums, Pydantic models, and explicit exceptions.

Most of these live in [`pyhulax.core`][pyhulax.core.Direction].

## Import Pattern

```python
from pyhulax.core import (
    Direction,
    FlipDirection,
    LEDMode,
    CommandResult,
    VelocityLevel,
    WiFiMode,
    MediaType,
    Vector3,
    Orientation,
    LEDConfig,
    FlightData,
    Obstacles,
    DroneState,
    AIResult,
    ColorResult,
    MediaFile,
    DroneConnectionError,
    NotReady,
    LowBattery,
    TelemetryUnavailable,
)
```

## Key Enums

Movement and control:

- [`Direction`][pyhulax.core.types.Direction]
- [`Rotation`][pyhulax.core.types.Rotation]
- [`FlipDirection`][pyhulax.core.types.FlipDirection]
- [`VelocityLevel`][pyhulax.core.types.VelocityLevel]
- [`TakeoffFlags`][pyhulax.core.types.TakeoffFlags]

LED and payload:

- [`LEDMode`][pyhulax.core.types.LEDMode]
- [`ClampMode`][pyhulax.core.types.ClampMode]
- [`ElectromagnetMode`][pyhulax.core.types.ElectromagnetMode]
- [`LaserMode`][pyhulax.core.types.LaserMode]

Vision and camera:

- [`VisionMode`][pyhulax.core.types.VisionMode]
- [`AIRecognitionTarget`][pyhulax.core.types.AIRecognitionTarget]
- [`CameraMode`][pyhulax.core.types.CameraMode]
- [`CameraPitchMode`][pyhulax.core.types.CameraPitchMode]
- [`VideoMode`][pyhulax.core.types.VideoMode]
- [`VideoResolution`][pyhulax.core.types.VideoResolution]
- [`VideoStreamMode`][pyhulax.core.types.VideoStreamMode]
- [`QRLocalizationMode`][pyhulax.core.types.QRLocalizationMode]

Navigation and environment:

- [`BarrierMode`][pyhulax.core.types.BarrierMode]
- [`BarrierMask`][pyhulax.core.types.BarrierMask]
- [`LineColor`][pyhulax.core.types.LineColor]
- [`LineFollowResult`][pyhulax.core.types.LineFollowResult]

System:

- [`DroneStatus`][pyhulax.core.types.DroneStatus]
- [`CommandResult`][pyhulax.core.types.CommandResult]
- [`MediaType`][pyhulax.core.types.MediaType]
- [`WiFiMode`][pyhulax.core.types.WiFiMode]

## Key Models

### Geometry and State

- [`Vector3`][pyhulax.core.models.Vector3]
- [`Orientation`][pyhulax.core.models.Orientation]
- [`DroneState`][pyhulax.core.models.DroneState]
- [`FlightData`][pyhulax.core.models.FlightData]
- [`Obstacles`][pyhulax.core.models.Obstacles]

### Visual Results

- [`AIResult`][pyhulax.core.models.AIResult]
- [`ColorResult`][pyhulax.core.models.ColorResult]
- [`MediaFile`][pyhulax.core.models.MediaFile]

### LED

- `LEDConfig`
- [`LEDColor`][pyhulax.core.models.LEDColor]

## Common Model Usage

```python
from pyhulax.core import LEDConfig, LEDMode, Direction

led = LEDConfig.rgb(255, 0, 0, mode=LEDMode.BLINK)
drone.move(Direction.FORWARD, 100, led=led)
```

```python
state = drone.get_state()
print(state.position.x, state.position.y, state.battery_percent)
```

Telemetry model access:

```python
flight = drone.get_flight_data()

print(flight.position.x, flight.position.y, flight.position.z)
print(flight.velocity)
print(flight.orientation.yaw)
print(flight.altitude_tof)
```

LED patterns from a real wheel smoke test:

```python
from pyhulax.core import LEDConfig, LEDMode

takeoff_led = LEDConfig(r=125, g=125, b=125, mode=LEDMode.BLINK)
hover_led = LEDConfig(r=255, g=0, b=0, mode=LEDMode.CONSTANT)
land_led = LEDConfig(r=0, g=0, b=255, mode=LEDMode.CONSTANT)

drone.takeoff(led=takeoff_led)
drone.hover(led=hover_led)
drone.land(led=land_led)
```

## Exceptions

Base class:

- [`DroneError`][pyhulax.core.exceptions.DroneError]

Operational exceptions:

- [`DroneConnectionError`][pyhulax.core.exceptions.DroneConnectionError]
- [`CommandTimeout`][pyhulax.core.exceptions.CommandTimeout]
- [`CommandRejected`][pyhulax.core.exceptions.CommandRejected]
- [`NotReady`][pyhulax.core.exceptions.NotReady]
- [`LowBattery`][pyhulax.core.exceptions.LowBattery]
- [`TelemetryUnavailable`][pyhulax.core.exceptions.TelemetryUnavailable]

Typical exception handling:

```python
from pyhulax import DroneAPI
from pyhulax.core import DroneConnectionError, LowBattery, NotReady

try:
    with DroneAPI() as drone:
        drone.connect()
        drone.takeoff()
except DroneConnectionError:
    print("Connection failed")
except LowBattery:
    print("Battery too low to continue")
except NotReady:
    print("Drone rejected the command")
```

Additional validation exceptions exist in [`pyhulax.core.exceptions`][pyhulax.core.exceptions.DroneError]:

- [`InvalidParameter`][pyhulax.core.exceptions.InvalidParameter]
- [`OperationInProgress`][pyhulax.core.exceptions.OperationInProgress]

## Controller Types

[`pyhulax.control`][pyhulax.control.ManualFlightController] exports:

- [`ManualFlightController`][pyhulax.control.ManualFlightController]
- [`ControllerConfig`][pyhulax.control.ControllerConfig]
- [`ControllerResult`][pyhulax.control.ControllerResult]
- [`FlightState`][pyhulax.control.FlightState]

Typical usage:

```python
from pyhulax.control import ControllerConfig

ctrl = drone.create_flight_controller(
    ControllerConfig(kp_xy=2.5, kp_z=3.5)
)
```

And then:

```python
result = ctrl.fly_to(x=100, y=150, z=100, yaw=90)
print(result.success, result.reason)
```

## Video Types

[`pyhulax.video`][pyhulax.video] exports core stream data types even without optional backends:

- [`Frame`][pyhulax.video.types.Frame]
- [`Detection`][pyhulax.video.types.Detection]
- [`BoundingBox`][pyhulax.video.types.BoundingBox]
- [`FrameCallback`][pyhulax.video.types.FrameCallback]
- [`StreamConfig`][pyhulax.video.types.StreamConfig]
- [`StreamState`][pyhulax.video.types.StreamState]

These become especially useful once the `video`, `vision`, or `web` extras are installed.

Example callback signature:

```python
from pyhulax.video import Frame

def annotate(frame: Frame) -> Frame:
    frame.metadata["pipeline"] = "demo"
    return frame
```
