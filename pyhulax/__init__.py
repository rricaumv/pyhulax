"""
Modern typed API for HG-Fly F09-lite (Hula) drone control.

Provides type-safe interface with proper error handling, replacing the legacy UserApi.

Example usage:
```python
from pyhulax import DroneAPI
from pyhulax.core import Direction, LEDConfig, LEDMode

with DroneAPI() as drone:
    drone.connect("192.168.100.1")
    drone.takeoff()
    drone.move(Direction.FORWARD, 100)
    drone.rotate(90)  # Turn left 90 degrees
    drone.move_to(50, 100, 0)  # Move to relative position
    drone.land()
```

Or with logging:
```python
from pyhulax.logging import SQLiteLogger

drone = DroneAPI(flight_logger=SQLiteLogger("flights.db"))
drone.connect()
# ... flight operations are automatically logged
```
"""

from __future__ import annotations

import logging
import shutil
import sysconfig
import time
from importlib.machinery import PathFinder
from importlib.util import module_from_spec
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import requests

ManualFlyFrameCallback = Callable[[float, float, float, float, int, bool], None]

from pyhulax.core.types import (
    Direction,
    FlipDirection,
    LEDMode,
    DroneStatus,
    CommandResult,
    VisionMode,
    AIRecognitionTarget,
    ClampMode,
    ElectromagnetMode,
    VideoMode,
    BarrierMode,
    LineFollowResult,
    LineColor,
    LaserMode,
    CameraPitchMode,
    VideoStreamMode,
    QRLocalizationMode,
    MediaType,
    VelocityLevel,
    BarrierMask,
    VideoResolution,
    TakeoffFlags,
    WiFiMode,
)
from pyhulax.core.models import (
    Vector3,
    Orientation,
    LEDConfig,
    FlightData,
    Obstacles,
    DroneState,
    AIResult,
    ColorResult,
    MediaFile,
)
from pyhulax.core.exceptions import (
    DroneConnectionError,
    CommandTimeout,
    NotReady,
    LowBattery,
    TelemetryUnavailable,
)
from pyhulax.config import (
    BatteryConfig,
    ControllerConfig as ControllerDefaultsConfig,
    DroneConfig,
    DronePhysicsConfig,
    FlightConfig,
    MediaConfig,
    NetworkConfig,
    ProtocolConfig,
    TimeoutConfig,
    VideoConfig,
    resolve_config,
)
from pyhulax.fylo.controlserver import Controlserver
from pyhulax.control import ManualFlightController, ControllerConfig as FlightControllerConfig
from pyhulax.logging.command_logger import CommandLogger

if TYPE_CHECKING:
    from pyhulax.logging.base import FlightLogger
    from pyhulax.video.stream import VideoStream

if getattr(logging, "__name__", "") != "logging":
    _logging_spec = PathFinder.find_spec("logging", [sysconfig.get_paths()["stdlib"]])
    if _logging_spec is None or _logging_spec.loader is None:
        raise ImportError("Unable to resolve the standard library logging module")
    logging = module_from_spec(_logging_spec)
    _logging_spec.loader.exec_module(logging)

logger = logging.getLogger(__name__)


class DroneAPI:
    """
    Type-safe API for F09-lite drone control.

    Wraps the legacy Controlserver with proper types, validation, and error handling.
    All telemetry methods return immediately without artificial delays.
    """

    # Default media storage directories
    DEFAULT_MEDIA_DIR = Path("media")
    DEFAULT_PHOTO_DIR = DEFAULT_MEDIA_DIR / "photos"
    DEFAULT_VIDEO_DIR = DEFAULT_MEDIA_DIR / "videos"
    DEFAULT_LOG_DIR = DEFAULT_MEDIA_DIR / "logs"

    def __init__(
        self,
        config: DroneConfig | None = None,
        enable_logging: bool = True,
        flight_logger: FlightLogger | None = None,
        battery_threshold: int | None = None,
        media_dir: Path | str | None = None,
        enable_file_logging: bool = True,
        file_log_dir: str = "logs",
        enable_command_logging: bool = True,
        command_log_dir: str = "logs",
        drone_id: int | None = None,
    ):
        """
        Initialize DroneAPI.

        Args:
            config: Optional runtime settings for network, protocol, video, and controller defaults.
            enable_logging: Enable console logging
            flight_logger: Optional flight logger for database logging
            battery_threshold: Low battery warning threshold (%). Defaults to config battery critical threshold.
            media_dir: Override the configured base directory for downloaded media
            enable_file_logging: Enable JSONL file logging for all MAVLink messages
            file_log_dir: Directory for JSONL log files (default: ./logs)
            enable_command_logging: Enable JSONL logging of all API commands
            command_log_dir: Directory for command log files (default: ./logs)
            drone_id: Optional explicit drone id for this connection. When omitted
                the id is discovered automatically from the drone's telemetry.
                Pin this when controlling multiple drones concurrently so each
                connection's commands and telemetry are unambiguously scoped.
        """
        self._config = resolve_config(config)
        self._server = Controlserver(runtime_config=self._config, drone_id=drone_id)
        self._connected = False
        self._flight_logger = flight_logger
        self._session_id: str | None = None
        self._battery_threshold = (
            battery_threshold
            if battery_threshold is not None
            else self._config.battery.critical_threshold
        )
        self._drone_ip: str = self._config.network.drone_ip
        self._enable_file_logging = enable_file_logging
        self._file_log_dir = file_log_dir

        # Command logging
        self._command_logger: CommandLogger | None = None
        if enable_command_logging:
            self._command_logger = CommandLogger(
                log_dir=command_log_dir,
                prefix="commands",
            )

        # Media directories
        self._media_dir = self._resolve_base_media_dir(media_dir)
        self._photo_dir = self._resolve_media_subdir(
            self._config.media.photo_dir,
            default_name="photos",
        )
        self._video_dir = self._resolve_media_subdir(
            self._config.media.video_dir,
            default_name="videos",
        )
        self._log_dir = self._resolve_media_subdir(
            self._config.media.log_dir,
            default_name="logs",
        )

        if enable_logging:
            self._log = logger
        else:
            self._log = None

    def __enter__(self) -> DroneAPI:
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.disconnect()
        return False

    def _resolve_base_media_dir(self, media_dir_override: Path | str | None) -> Path:
        """Resolve the root media directory using call-time override or config defaults."""
        if media_dir_override is not None:
            return Path(media_dir_override)
        return Path(self._config.media.base_dir)

    def _resolve_media_subdir(
        self,
        configured_path: Path | str | None,
        *,
        default_name: str,
    ) -> Path:
        """Resolve a media subdirectory relative to the base media directory unless absolute."""
        if configured_path is None:
            return self._media_dir / default_name

        path = Path(configured_path)
        if path.is_absolute():
            return path
        return self._media_dir / path

    def __getattribute__(self, name: str):
        """Intercept method calls to add command logging."""
        attr = super().__getattribute__(name)

        # Skip private/magic attributes, non-callables, and specific methods
        if (
            name.startswith("_")
            or not callable(attr)
            or name in ("__enter__", "__exit__", "__getattribute__")
        ):
            return attr

        # Get command logger (avoid recursion by using super())
        try:
            cmd_logger = super().__getattribute__("_command_logger")
        except AttributeError:
            return attr

        if cmd_logger is None:
            return attr

        # Wrap the method with logging
        return cmd_logger.log(attr)

    # ==================== Connection Management ====================

    def connect(self, ip: str | None = None, timeout: float = 5.0) -> None:
        """
        Connect to drone.

        Args:
            ip: Drone IP address. Uses the configured default if None.
            timeout: Connection timeout in seconds (unused currently).

        Raises:
            DroneConnectionError: If connection fails.
        """
        result = self._server.connect(
            ip,
            enable_file_logging=self._enable_file_logging,
            log_dir=self._file_log_dir,
        )
        if not result:
            raise DroneConnectionError("Failed to connect to drone", ip=ip)
        self._connected = True
        self._drone_ip = ip or self._config.network.drone_ip
        if self._log:
            self._log.info(f"Connected to drone at {self._drone_ip}")

    def disconnect(self) -> None:
        """Disconnect from drone and release resources."""
        if self._connected:
            try:
                self._server.disconnect()
            except Exception as e:
                if self._log:
                    self._log.warning(f"Error during disconnect: {e}")
            finally:
                self._connected = False
                if self._log:
                    self._log.info("Disconnected from drone")

        # Close command logger
        if self._command_logger:
            self._command_logger.close()

    @property
    def is_connected(self) -> bool:
        """Check if connected to drone."""
        return self._connected

    @property
    def config(self) -> DroneConfig:
        """Resolved runtime configuration for this API instance."""
        return self._config

    @property
    def default_ip(self) -> str:
        """Configured default drone IP address."""
        return self._config.network.drone_ip

    def robust_connect(
        self,
        ip: str | None = None,
        timeout: float = 5.0,
        verbose: bool = True,
    ) -> bool:
        """
        Connect to drone with robust error handling and diagnostics.

        Unlike connect(), this method doesn't raise exceptions on failure.
        Instead, it prints diagnostic information and returns a boolean.

        Args:
            ip: Drone IP address. Uses the configured default if None.
            timeout: Connection timeout in seconds (unused currently).
            verbose: Print diagnostic messages.

        Returns:
            True if connection successful, False otherwise.

        Example:
        ```python
        if not drone.robust_connect("192.168.100.1"):
            print("Check WiFi connection")
            sys.exit(1)
        ```
        """
        try:
            if verbose:
                print(f"Connecting to drone at {ip or self.default_ip}...")

            result = self._server.connect(
                ip,
                enable_file_logging=self._enable_file_logging,
                log_dir=self._file_log_dir,
            )

            if result:
                self._connected = True
                self._drone_ip = ip or self._config.network.drone_ip
                if verbose:
                    print("\u2713 Connection established")
                if self._log:
                    self._log.info(f"Connected to drone at {self._drone_ip}")
                return True
            else:
                if verbose:
                    print("\u2717 Connection failed - no response from drone")
                return False

        except Exception as e:
            if verbose:
                print(f"\u2717 Connection error: {e}")
                print("Troubleshooting:")
                print(f"  1. Verify drone is at IP: {ip or self.default_ip}")
                print("  2. Check WiFi connection to drone network")
                print("  3. Ensure drone is powered and in AP mode")
            return False

    # ==================== Unified Movement ====================

    def move(
        self,
        direction: Direction,
        distance_cm: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
        speed: VelocityLevel | int = VelocityLevel.ZOOM,
    ) -> CommandResult:
        """
        Move drone in specified direction.

        This is the unified movement API replacing separate forward/back/left/right methods.

        Args:
            direction: Movement direction (FORWARD, BACK, LEFT, RIGHT, UP, DOWN)
            distance_cm: Distance in centimeters (positive value)
            led: Optional LED configuration during movement
            blocking: If True (default), wait for command completion. If False, return immediately.
            speed: Movement speed - VelocityLevel.SLOW, MEDIUM, ZOOM, or TURBO.
                   Controls position controller P-gain (lower value = higher gain = faster).
                   Default is ZOOM (100).

        Returns:
            CommandResult indicating success/failure

        Raises:
            NotReady: If drone not connected or not ready
            LowBattery: If battery below threshold

        Example:
        ```python
        drone.move(Direction.FORWARD, 100)  # Move forward 100cm (default ZOOM speed)
        drone.move(Direction.FORWARD, 100, speed=VelocityLevel.TURBO)  # Very fast
        drone.move(Direction.LEFT, 50, speed=VelocityLevel.SLOW)  # Slow, smooth
        ```
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        # VelocityLevel enum values are already in cm/s (100, 200, 300)
        speed_cms = int(speed)

        dispatch = {
            Direction.FORWARD: self._server.single_fly_forward,
            Direction.BACK: self._server.single_fly_back,
            Direction.LEFT: self._server.single_fly_left,
            Direction.RIGHT: self._server.single_fly_right,
            Direction.UP: self._server.single_fly_up,
            Direction.DOWN: self._server.single_fly_down,
        }

        handler = dispatch.get(direction)
        if handler is None:
            raise ValueError(f"Invalid direction: {direction}")

        # UP/DOWN use 'height' parameter, others use 'distance'
        if direction in (Direction.UP, Direction.DOWN):
            result = handler(abs(distance_cm), led_param, blocking=blocking, speed=speed_cms)
        else:
            result = handler(abs(distance_cm), led_param, blocking=blocking, speed=speed_cms)

        return self._parse_result(result)

    def rotate(
        self,
        angle_degrees: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Rotate drone by specified angle.

        Args:
            angle_degrees: Rotation angle in degrees.
                          Positive = counter-clockwise (left)
                          Negative = clockwise (right)
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0

        if angle_degrees >= 0:
            result = self._server.single_fly_turnleft(angle_degrees, led_param, blocking=blocking)
        else:
            result = self._server.single_fly_turnright(angle_degrees, led_param, blocking=blocking)

        return self._parse_result(result)

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
        speed: VelocityLevel | int = VelocityLevel.ZOOM,
    ) -> CommandResult:
        """
        Move to position via straight line flight.

        Position interpretation depends on QR localization mode:

        - QR localization ENABLED (set_qr_localization(True)):
          Coordinates are ABSOLUTE positions in the QR code coordinate system.
          The drone flies to the exact (x, y, z) position on the QR mat.

        - QR localization DISABLED (default):
          Coordinates are relative to the takeoff position (which is 0, 0, 0).
          Effectively works as absolute positioning from the takeoff origin.

        Args:
            x: X position in cm (right is positive)
            y: Y position in cm (forward is positive)
            z: Z position in cm (up is positive, relative to ground)
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.
            speed: Movement speed - VelocityLevel.SLOW, MEDIUM, ZOOM, or TURBO.
                   Controls position controller P-gain (lower value = higher gain = faster).
                   Default is ZOOM (100).

        Returns:
            CommandResult

        Example:
        ```python
        # With QR localization - fly to absolute position on QR mat
        drone.set_qr_localization(enabled=True)
        drone.move_to(x=200, y=350, z=120)  # Fly to (200, 350, 120) cm

        # Without QR localization - position relative to takeoff point
        drone.move_to(x=100, y=100, z=0)  # Fly to 100cm right, 100cm forward

        # With speed control
        drone.move_to(x=100, y=100, z=50, speed=VelocityLevel.TURBO)  # Very fast
        drone.move_to(x=100, y=100, z=50, speed=VelocityLevel.SLOW)  # Slow, smooth
        ```
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        # VelocityLevel enum values are already in cm/s (100, 200, 300)
        speed_cms = int(speed)
        result = self._server.single_fly_straight_flight(x, y, z, led_param, blocking=blocking, speed=speed_cms)
        return self._parse_result(result)

    def curve_to(
        self,
        x: float,
        y: float,
        z: float,
        ccw: bool = True,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Move to position along curved path.

        Like move_to(), position interpretation depends on QR localization mode:

        - QR enabled: Absolute position on QR mat
        - QR disabled: Position relative to takeoff origin (0, 0, 0)

        Args:
            x: X position in cm
            y: Y position in cm
            z: Z position in cm
            ccw: Counter-clockwise curve (True) or clockwise (False)
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_curvilinearFlight(ccw, x, y, z, led_param, blocking=blocking)
        return self._parse_result(result)

    def circle(
        self,
        radius_cm: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Fly in a circle around current position.

        Args:
            radius_cm: Circle radius in cm.
                      Positive = counter-clockwise
                      Negative = clockwise
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_radius_around(radius_cm, led_param, blocking=blocking)
        return self._parse_result(result)

    # ==================== Flight Control ====================

    def takeoff(
        self,
        height_cm: int = 100,
        led: LEDConfig | None = None,
        blocking: bool = True,
        flags: TakeoffFlags | int = TakeoffFlags.NONE,
    ) -> CommandResult:
        """
        Take off to specified hover height.

        Args:
            height_cm: Target hover height in centimeters (default 100)
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.
            flags: Takeoff behavior flags (TakeoffFlags enum or int bitmask):

                - TakeoffFlags.NONE (0): Normal takeoff (default)
                - TakeoffFlags.RESET_YAW (1): Reset yaw orientation on takeoff
                - TakeoffFlags.WITH_LOAD (2): Takeoff with load/clamp - may have different dynamics
                Flags can be combined: TakeoffFlags.RESET_YAW | TakeoffFlags.WITH_LOAD

        Returns:
            CommandResult

        Raises:
            NotReady: If drone not in ready state

        Example:
        ```python
        drone.takeoff()  # Normal takeoff to 100cm
        drone.takeoff(height_cm=80, flags=TakeoffFlags.WITH_LOAD)  # With load mode
        drone.takeoff(flags=TakeoffFlags.RESET_YAW | TakeoffFlags.WITH_LOAD)  # Combined
        ```
        """
        self._check_ready(check_flying=False)
        led_param = led.to_param4() if led else 0
        flags_int = int(flags)
        result = self._server.single_fly_takeoff(
            led_param, height=height_cm, flags=flags_int, blocking=blocking
        )
        return self._parse_result(result)

    def land(self, led: LEDConfig | None = None, blocking: bool = True) -> CommandResult:
        """
        Land the drone.

        Args:
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_touchdown(led_param, blocking=blocking)
        return self._parse_result(result)

    def hover(
        self,
        duration_seconds: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Hover in place for specified duration.

        Args:
            duration_seconds: Hover duration
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        # MAVLink struct requires integer duration
        result = self._server.single_fly_hover_flight(int(duration_seconds), led_param, blocking=blocking)
        return self._parse_result(result)

    def flip(
        self,
        direction: FlipDirection,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Perform a flip/somersault maneuver.

        Args:
            direction: Flip direction (FORWARD, BACK, LEFT, RIGHT)
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_somersault(direction.value, led_param, blocking=blocking)
        return self._parse_result(result)

    def bounce(
        self,
        frequency: int,
        height_cm: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Perform bouncing motion.

        Args:
            frequency: Bounce frequency
            height_cm: Bounce height in cm
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_bounce(frequency, height_cm, led_param, blocking=blocking)
        return self._parse_result(result)

    def spin(
        self,
        rotations: float,
        led: LEDConfig | None = None,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Perform 360-degree rotation(s).

        Args:
            rotations: Number of rotations.
                      Positive = counter-clockwise
                      Negative = clockwise
            led: Optional LED configuration
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        self._check_ready()
        led_param = led.to_param4() if led else 0
        result = self._server.single_fly_autogyration360(rotations, led_param, blocking=blocking)
        return self._parse_result(result)

    def arm(self) -> CommandResult:
        """
        Arm motors (low speed rotation).

        Returns:
            CommandResult
        """
        self._check_ready(check_flying=False)
        result = self._server.plane_fly_arm()
        return self._parse_result(result)

    def disarm(self) -> CommandResult:
        """
        Disarm motors.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.plane_fly_disarm()
        return self._parse_result(result)

    # ==================== Telemetry (No Sleeps!) ====================

    def get_state(self) -> DroneState:
        """
        Get complete drone state snapshot.

        Returns:
            DroneState with all telemetry data
        """
        raw_flight = self._server.get_plane_data("flight_data")
        raw_heartbeat = self._server.get_plane_data("heartbeat")

        flight_data = None
        if raw_flight is not None:
            flight_data = FlightData.from_mavlink(raw_flight)

        status = DroneStatus.DISCONNECTED
        if raw_heartbeat is not None:
            # Map drone_status from heartbeat to DroneStatus enum
            if raw_heartbeat.drone_status == 2:
                status = DroneStatus.READY
            elif raw_heartbeat.drone_status == 3:
                status = DroneStatus.FLYING
            else:
                status = DroneStatus.CONNECTED

        obstacles = Obstacles()
        if raw_flight is not None:
            obstacles = Obstacles.from_bitmask(raw_flight.barrier)

        return DroneState(
            status=status,
            flight_data=flight_data,
            obstacles=obstacles,
            drone_id=self._server.get_plane_id() if self._connected else None,
            is_connected=self._connected,
            last_heartbeat=None,  # TODO: track timestamp
        )

    def get_position(self) -> Vector3:
        """
        Get current position.

        Returns:
            Position vector in cm

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("position")
        return Vector3(x=float(data.x), y=float(data.y), z=float(data.z))

    def get_orientation(self) -> Orientation:
        """
        Get current orientation (yaw, pitch, roll).

        Returns:
            Orientation in degrees

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("orientation")
        return Orientation(
            yaw=data.yaw / 100.0,
            pitch=data.pitch / 100.0,
            roll=data.roll / 100.0,
        )

    def get_battery(self) -> int:
        """
        Get battery percentage.

        Returns:
            Battery level (0-100)

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("battery")
        return int(data.battery_volumn)

    def get_altitude(self) -> float:
        """
        Get ToF altitude.

        Returns:
            Altitude in cm

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("altitude")
        return float(data.distance)

    def get_velocity(self) -> Vector3:
        """
        Get current velocity.

        Returns:
            Velocity vector in cm/s

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("velocity")
        return Vector3(x=float(data.vel_x), y=float(data.vel_y), z=float(data.vel_z))

    def get_acceleration(self) -> Vector3:
        """
        Get current acceleration.

        Returns:
            Acceleration vector in cm/s^2

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("acceleration")
        return Vector3(x=float(data.accx), y=float(data.accy), z=float(data.accz))

    def get_obstacles(self) -> Obstacles:
        """
        Get obstacle detection state.

        Returns:
            Obstacles indicating which directions are blocked
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            return Obstacles()
        return Obstacles.from_bitmask(data.barrier)

    def get_flight_data(self) -> FlightData:
        """
        Get complete flight telemetry snapshot.

        Returns:
            FlightData with all sensor values

        Raises:
            TelemetryUnavailable: If no telemetry data
        """
        data = self._server.get_plane_data("flight_data")
        if data is None:
            raise TelemetryUnavailable("flight_data")
        return FlightData.from_mavlink(data)

    def get_drone_id(self) -> int | None:
        """
        Get drone ID.

        Returns:
            Drone ID as integer, or None if not connected
        """
        if not self._connected:
            return None
        return self._server.get_plane_id()

    # ==================== Vision / AI ====================

    def recognize_target(self, target: AIRecognitionTarget | int) -> AIResult:
        """
        Attempt to recognize a visual target (digit, arrow, letter).

        Args:
            target: Target type to recognize. Use AIRecognitionTarget enum:

                - AIRecognitionTarget.DIGIT_0 through DIGIT_9 for digits
                - AIRecognitionTarget.ARROW_LEFT/RIGHT/UP/DOWN for arrows
                - AIRecognitionTarget.END_TASK for end marker
                - Integer 65-90 for letters A-Z (ASCII codes)

        Returns:
            AIResult with recognition data

        Example:
        ```python
        result = drone.recognize_target(AIRecognitionTarget.DIGIT_5)
        result = drone.recognize_target(AIRecognitionTarget.ARROW_LEFT)
        result = drone.recognize_target(65)  # Letter 'A'
        ```
        """
        mode = int(target) if isinstance(target, AIRecognitionTarget) else target
        result = self._server.single_fly_AiIdentifies(mode)
        return AIResult.from_dict(result)

    def recognize_qr(
        self,
        qr_id: int,
        mode: VisionMode = VisionMode.OPTICAL_FLOW,
        timeout: float = 10.0,
        search_radius: int = 0,
    ) -> AIResult:
        """
        Recognize and align to QR code.

        Args:
            qr_id: QR code ID to find
            mode: Camera mode (optical flow or front camera)
            timeout: Search timeout in seconds
            search_radius: Search radius

        Returns:
            AIResult with QR position data
        """
        result = self._server.single_fly_Qrcode_align(mode.value, timeout, search_radius, qr_id)
        return AIResult(
            success=bool(result.get("result", False)),
            position=Vector3(
                x=float(result.get("x", 0)),
                y=float(result.get("y", 0)),
                z=float(result.get("z", 0)),
            ) if result.get("result") else None,
            angle=float(result.get("yaw", 0)) if result.get("yaw") else None,
            qr_id=result.get("qr_id"),
        )

    def track_qr(
        self,
        qr_id: int,
        duration: float,
        mode: VisionMode = VisionMode.OPTICAL_FLOW,
    ) -> AIResult:
        """
        Track QR code for specified duration.

        Args:
            qr_id: QR code ID to track
            duration: Tracking duration in seconds
            mode: Camera mode

        Returns:
            AIResult with tracking data
        """
        result = self._server.single_fly_Qrcode_tracking(mode.value, qr_id, duration)
        return AIResult.from_dict(result)

    def detect_qr(
        self,
        qr_id: int,
        mode: VisionMode = VisionMode.OPTICAL_FLOW,
    ) -> AIResult:
        """
        Detect and recognize QR code without alignment.

        Unlike recognize_qr() which aligns the drone to the QR code,
        this method only detects and returns position information.

        Args:
            qr_id: QR code ID to detect (0-9)
            mode: Camera mode (OPTICAL_FLOW for downward, FRONT_CAMERA for forward)

        Returns:
            AIResult with:

                - success: Whether QR was detected
                - position: Distance from drone to QR (x, y, z in cm)
                - angle: Yaw angle between drone and QR
                - qr_id: ID of detected QR code

        Example:
        ```python
        # Detect QR code 5 using optical flow camera
        result = drone.detect_qr(5, VisionMode.OPTICAL_FLOW)
        if result.success:
            print(f"QR {result.qr_id} at {result.position}")
        ```
        """
        if mode == VisionMode.OPTICAL_FLOW:
            # Optical flow detection: mode=2 in Qrcode_align
            result = self._server.single_fly_Qrcode_align(2, 0, 0, qr_id)
        else:
            # Front camera detection: tracking_type=2 in Qrcode_tracking
            result = self._server.single_fly_Qrcode_tracking(qr_id, 2, 0)

        return AIResult(
            success=bool(result.get("result", False)),
            position=Vector3(
                x=float(result.get("x", 0)),
                y=float(result.get("y", 0)),
                z=float(result.get("z", 0)),
            ) if result.get("result") else None,
            angle=float(result.get("yaw", 0)) if result.get("yaw") else None,
            qr_id=result.get("qr_id"),
        )

    def get_color(self, mode: int = 1) -> ColorResult:
        """
        Get dominant color from camera.

        Args:
            mode: Color recognition mode

        Returns:
            ColorResult with RGB values
        """
        result = self._server.single_fly_getColor(mode)
        if result is None:
            return ColorResult(success=False)
        return ColorResult.from_response(
            r=result.get("r", 0),
            g=result.get("g", 0),
            b=result.get("b", 0),
            state=result.get("state", 0),
        )

    # ==================== LED / Peripherals ====================

    def set_led(
        self,
        led_or_r: LEDConfig | int,
        g: int = 0,
        b: int = 0,
        mode: LEDMode = LEDMode.CONSTANT,
        duration: float = 0,
    ) -> CommandResult:
        """
        Set LED color and mode.

        Can be called with LEDConfig or individual RGB values:
        ```python
        drone.set_led(LEDColor.RED)
        drone.set_led(LEDConfig.rgb(255, 128, 0, LEDMode.BLINK))
        drone.set_led(255, 0, 0)  # Legacy: r, g, b
        ```

        Args:
            led_or_r: LEDConfig object, or red channel (0-255) for legacy usage
            g: Green channel (0-255) - only used if led_or_r is int
            b: Blue channel (0-255) - only used if led_or_r is int
            mode: LED mode - only used if led_or_r is int
            duration: Effect duration in seconds (0 = indefinite)

        Returns:
            CommandResult

        Example:
        ```python
        # Using LEDConfig (preferred)
        drone.set_led(LEDColor.BLUE)
        drone.set_led(LEDColor.RED.with_mode(LEDMode.BLINK))
        drone.set_led(LEDConfig.rgb(128, 64, 255))

        # Using individual values (legacy)
        drone.set_led(255, 0, 0, LEDMode.CONSTANT)
        ```
        """
        if isinstance(led_or_r, LEDConfig):
            r, g, b = led_or_r.r, led_or_r.g, led_or_r.b
            mode = led_or_r.mode
        else:
            r = led_or_r
        result = self._server.single_fly_lamplight(r, g, b, duration, mode.value)
        return self._parse_result(result)

    def enable_led(self, blocking: bool = True) -> CommandResult:
        """
        Enable LEDs.

        Re-enables LEDs after they have been disabled with disable_led().
        Does not change the current LED color/mode - just makes them visible.

        Args:
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        Example:
        ```python
        drone.set_led(255, 0, 0)  # Set red
        drone.disable_led()       # Turn off
        drone.enable_led()        # Red is back
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.enable_led(blocking=blocking)
        return self._parse_result(result)

    def disable_led(self, blocking: bool = True) -> CommandResult:
        """
        Disable LEDs.

        Turns off all LEDs without clearing the color/mode settings.
        Use enable_led() to turn them back on.

        Args:
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        Example:
        ```python
        drone.set_led(0, 255, 0)  # Set green
        drone.disable_led()       # LEDs off
        # ... later ...
        drone.enable_led()        # Green is back
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.disable_led(blocking=blocking)
        return self._parse_result(result)

    def cancel_rgb_animation(self, blocking: bool = True) -> CommandResult:
        """
        Cancel current RGB animation.

        Stops any running LED animation/effect and returns LEDs to static state.
        Useful for interrupting breathing, blinking, or other animated effects.

        Args:
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        Example:
        ```python
        drone.set_led(255, 0, 0, mode=LEDMode.BREATHING)  # Start breathing
        time.sleep(5)
        drone.cancel_rgb_animation()  # Stop animation
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.cancel_rgb(blocking=blocking)
        return self._parse_result(result)

    def set_rgb_brightness(self, brightness: int, blocking: bool = True) -> CommandResult:
        """
        Set RGB LED brightness.

        Args:
            brightness: Brightness level (firmware-dependent range)
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_rgb_brightness(brightness, blocking=blocking)
        return self._parse_result(result)

    # ==================== Configuration ====================

    def vertical_circle(self, radius_cm: int, blocking: bool = True) -> CommandResult:
        """
        Perform vertical circle maneuver.

        Args:
            radius_cm: Circle radius in cm (positive=CCW, negative=CW)
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        Note:
            Requires minimum altitude of 0.35m
        """
        self._check_ready()
        result = self._server.vertical_circle(radius_cm, blocking=blocking)
        return self._parse_result(result)

    def set_avoidance_direction(
        self,
        direction: Direction,
        distance_cm: int = 0,
        barrier_mask: BarrierMask | int = BarrierMask.ALL,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Set conditional avoidance behavior with directional control.

        When obstacle is detected (checked against barrier_mask), drone moves
        in the specified direction. This is a conditional move, it only executes
        if an obstacle is actually detected by the IR/ToF sensors.

        Args:
            direction: Direction to move when obstacle detected (FORWARD, BACK, LEFT, RIGHT, UP, DOWN)
            distance_cm: Avoidance distance in cm
            barrier_mask: Which sensors trigger avoidance. Use BarrierMask enum:

                - BarrierMask.FRONT, BACK, LEFT, RIGHT, UP, DOWN
                - BarrierMask.HORIZONTAL (front + back + left + right)
                - BarrierMask.ALL (default, all sensors)
                - Combine with |: BarrierMask.FRONT | BarrierMask.BACK
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        Example:
        ```python
        # Move back 50cm if obstacle detected in front
        drone.set_avoidance_direction(Direction.BACK, 50, BarrierMask.FRONT)

        # Move left 30cm if front or right obstacle detected
        drone.set_avoidance_direction(Direction.LEFT, 30, BarrierMask.FRONT | BarrierMask.RIGHT)

        # Move up if any horizontal obstacle
        drone.set_avoidance_direction(Direction.UP, 20, BarrierMask.HORIZONTAL)
        ```
        """
        self._check_ready()
        # Map direction to x/y parameters based on firmware expectations
        x = distance_cm if direction in (Direction.FORWARD, Direction.BACK) else 0
        y = distance_cm if direction in (Direction.LEFT, Direction.RIGHT) else 0
        mask_int = int(barrier_mask)
        result = self._server.set_avoidance(int(direction), mask_int, x, y, blocking=blocking)
        return self._parse_result(result)

    def get_product_id(self) -> CommandResult:
        """
        Request product ID / autopilot version from drone.

        Triggers the drone to send autopilot version information.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.get_product_id()
        return self._parse_result(result)

    def set_velocity_level(
        self,
        level: VelocityLevel | int,
        horizontal_vel: int = 0,
        blocking: bool = True,
    ) -> CommandResult:
        """
        Set manual/RC control velocity level.

        IMPORTANT:
            This command sets the velocity limit for MANUAL RC/joystick control only.
            It does NOT affect the speed of API movement commands like move() or move_to().

            To control the speed of API movement commands, use the `speed` parameter directly:
            ```python
            drone.move(Direction.FORWARD, 100, speed=VelocityLevel.TURBO)
            drone.move_to(x=100, y=100, z=50, speed=VelocityLevel.ZOOM)
            ```

        This command affects:

            - RC/joystick manual control speed limits
            - manual_vel_max, manual_vel_z_up_max, manual_vel_z_down_max in firmware

        Args:
            level: Velocity in cm/s (0-300). Use integer values directly:

                - 100 = 1.0 m/s (slow RC)
                - 200 = 2.0 m/s (medium RC)
                - 300 = 3.0 m/s (fast RC)
                NOTE: Do NOT use VelocityLevel enum here - it's designed for move() P-gain
                control, not RC velocity mapping.
            horizontal_vel: Optional horizontal velocity override in cm/s (firmware-specific)
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult

        See Also:
            move() and move_to() - use their `speed` parameter for API movement speed control
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        level_int = int(level) if isinstance(level, VelocityLevel) else level
        result = self._server.set_velocity(level_int, horizontal_vel, blocking=blocking)
        return self._parse_result(result)

    def set_yaw_rate_level(self, level: int, blocking: bool = True) -> CommandResult:
        """
        Set yaw rotation rate level.

        Args:
            level: Yaw rate level
            blocking: If True (default), wait for command completion. If False, return immediately.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_yawrate(level, blocking=blocking)
        return self._parse_result(result)

    def enable_battery_failsafe(self) -> CommandResult:
        """
        Enable battery failsafe.

        When enabled, drone will auto-land when battery reaches critical level.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.enable_battery_failsafe()
        return self._parse_result(result)

    def disable_battery_failsafe(self) -> CommandResult:
        """
        Disable battery failsafe.

        WARNING:
            Disabling battery failsafe may cause drone to fall if battery is depleted during flight.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.disable_battery_failsafe()
        return self._parse_result(result)

    def set_parameters(
        self,
        velocity: int = 0,
        yaw_rate: int = 0,
        brightness: int = 0,
        avoidance: bool = False,
        battery_failsafe: bool = False,
        fast_land: bool = False,
    ) -> CommandResult:
        """
        Set multiple drone parameters in a single command.

        This is more efficient than calling individual setters when
        configuring multiple parameters at once.

        Args:
            velocity: Velocity level (0-3, where 0=no change)
            yaw_rate: Yaw rate level
            brightness: RGB brightness level
            avoidance: Enable obstacle avoidance
            battery_failsafe: Enable battery failsafe
            fast_land: Enable fast landing speed

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_parameter(
            velocity=velocity,
            yaw_rate=yaw_rate,
            brightness=brightness,
            avoidance=avoidance,
            battery_failsafe=battery_failsafe,
            fast_land=fast_land,
        )
        return self._parse_result(result)

    def set_operate_status(self, status: int) -> CommandResult:
        """
        Set formation operate status.

        Used for formation flight coordination.

        Args:
            status: Operate status value

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_operate(status)
        return self._parse_result(result)

    def set_land_speed(self, fast: bool = False) -> CommandResult:
        """
        Set landing speed.

        Args:
            fast: True for fast landing, False for slow/safe landing

        Returns:
            CommandResult

        Note:
            Fast landing descends more quickly but may be less stable.
            Use slow landing for precision or when carrying payload.
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_land_speed(fast)
        return self._parse_result(result)

    def set_electromagnet(self, on: bool) -> CommandResult:
        """
        Control electromagnet.

        Args:
            on: True to activate, False to deactivate

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_CLAMP_ELECTROMAGNET (cmd=26)

            - type=3: OFF
            - type=4: ON
        """
        mode = ElectromagnetMode.ON if on else ElectromagnetMode.OFF
        # cmd=26 is CLAMP_ELECTROMAGNET (was incorrectly using 12)
        self._server.Plane_Linux_cmd(26, 1, mode.value, 0, 0)
        return CommandResult.SUCCESS

    def set_clamp(self, is_open: bool | None = None, angle: int | None = None) -> CommandResult:
        """
        Control gripper/clamp (servo/steering engine).

        Can be used in two modes:
        1. Simple open/close: set_clamp(is_open=True) or set_clamp(is_open=False)
        2. Angle control: set_clamp(angle=45) sets specific angle (0-180)

        Args:
            is_open: True to open, False to close (uses type 0/1)
            angle: Specific angle to set (0-180, uses type 2)

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_CLAMP_ELECTROMAGNET (cmd=26)

            - type=0: close
            - type=1: open
            - type=2: set angle
        """
        if angle is not None:
            # Angle mode: type=2, data=angle
            self._server.Plane_Linux_cmd(26, 1, ClampMode.SET_ANGLE.value, angle, 0)
        elif is_open is not None:
            # Simple open/close mode
            mode = ClampMode.OPEN if is_open else ClampMode.CLOSE
            self._server.Plane_Linux_cmd(26, 1, mode.value, 0, 0)
        else:
            raise ValueError("Must specify either is_open or angle")
        return CommandResult.SUCCESS

    def take_photo(
        self,
        download: bool = True,
        timeout: float = 5.0,
        save_path: Path | str | None = None,
    ) -> Path | None:
        """
        Capture photo and optionally download it.

        Note: The RTP video stream must be enabled before taking photos.
        Call `set_video_stream(True)` first if photos aren't being captured.

        Args:
            download: If True, wait for photo and download to media/photos/
            timeout: Max seconds to wait for photo to appear on drone
            save_path: Optional directory or exact file path for the downloaded photo.
                       If a directory is given, the drone's filename is preserved.

        Returns:
            Path to downloaded photo if download=True and successful, None otherwise.
            If download=False, returns None but photo is still captured on drone.

        Example:
        ```python
        # Enable video stream first (required for photo capture)
        drone.set_video_stream(True)

        # Capture and download
        photo_path = drone.take_photo()
        print(f"Photo saved to {photo_path}")

        # Capture only (manual download later)
        drone.take_photo(download=False)
        photos = drone.list_photos()

        # Capture and save to a custom path
        drone.take_photo(save_path="captures/latest.jpg")
        ```
        """
        # Get current photo count before capture
        #TODO First check if video stream is enabled? enabled if not, disable afterwards if we enabled it
        photos_before = []
        if download:
            try:
                photos_before = self.list_photos()
            except Exception:
                pass

        # Send capture command (cmd=5 is TAKE_PHOTO)
        # Note: RTP video stream should be enabled first via set_video_stream(True)
        self._server.Plane_Linux_cmd(5, 1, 0, 1, 0)

        if not download:
            return None

        # Wait for new photo to appear
        count_before = len(photos_before)
        start_time = time.time()

        while time.time() - start_time < timeout:
            time.sleep(0.5)
            try:
                photos_after = self.list_photos()
                if len(photos_after) > count_before:
                    # Found new photo - download it
                    newest = photos_after[0]  # List is sorted newest first
                    return self._download_media_to_path(newest, save_path)
            except Exception:
                pass

        if self._log:
            self._log.warning("Timeout waiting for photo to appear on drone")
        return None

    def set_video(self, recording: bool) -> CommandResult:
        """
        Start or stop video recording to SD card.

        Args:
            recording: True to start, False to stop

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_TAKE_VIDEO (cmd=6)

            - type=0: Begin recording
            - type=1: End recording
        """
        mode = VideoMode.START if recording else VideoMode.STOP
        # cmd=6 is TAKE_VIDEO (was incorrectly using 14)
        self._server.Plane_Linux_cmd(6, 1, mode.value, 0, 0)
        return CommandResult.SUCCESS

    def set_barrier_mode(self, enabled: bool) -> CommandResult:
        """
        Enable or disable obstacle avoidance.

        Args:
            enabled: True to enable, False to disable

        Returns:
            CommandResult
        """
        mode = BarrierMode.ENABLE if enabled else BarrierMode.DISABLE
        result = self._server.single_fly_barrier_aircraft(mode.value)
        return self._parse_result(result)

    # ==================== Line Following ====================

    def follow_line(
        self,
        distance_cm: float,
        line_color: LineColor = LineColor.BLACK,
    ) -> LineFollowResult:
        """
        Follow a line for specified distance.

        The drone follows a colored line on the ground using optical flow camera.
        Returns when distance is reached or an intersection is detected.

        Args:
            distance_cm: Distance to follow in centimeters
            line_color: Color of line to follow (BLACK or WHITE)

        Returns:
            LineFollowResult:

                - FAILED (0): Line following failed
                - SUCCESS (1): Successfully followed line for distance
                - INTERSECTION (2): Encountered intersection before reaching distance

        Example:
        ```python
        result = drone.follow_line(100, LineColor.BLACK)
        if result == LineFollowResult.INTERSECTION:
            # Handle intersection - check for arrows, decide direction
            pass
        ```
        """
        self._check_ready()
        # fun_id=0: move forward along line, ignoring intersections
        # tv parameter is unused (always 0)
        result = self._server.single_fly_Line_walking(0, int(distance_cm), 0, line_color.value)
        if result is None:
            return LineFollowResult.FAILED
        # Result dict contains 'result' key with 0/1/2
        result_code = result.get("result", 0)
        try:
            return LineFollowResult(result_code)
        except ValueError:
            return LineFollowResult.FAILED

    # ==================== Laser Control ====================

    def fire_laser(
        self,
        mode: LaserMode = LaserMode.SINGLE_SHOT,
        frequency: int = 10,
        ammo: int = 100,
    ) -> CommandResult:
        """
        Control laser firing.

        Args:
            mode: Laser mode (SINGLE_SHOT, BURST, CONTINUOUS, OFF, etc.)
            frequency: Firing frequency in shots/second (1-14), only for BURST mode
            ammo: Ammo capacity (1-255), only for BURST mode

        Returns:
            CommandResult

        Example:
        ```python
        # Single shot
        drone.fire_laser(LaserMode.SINGLE_SHOT)

        # Burst fire at 10 shots/sec with 50 ammo
        drone.fire_laser(LaserMode.BURST, frequency=10, ammo=50)

        # Stop firing
        drone.fire_laser(LaserMode.OFF)
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # Clamp values to valid ranges
        frequency = max(1, min(14, frequency))
        ammo = max(1, min(255, ammo))
        # Plane_Linux_cmd(7, ...) is laser control
        self._server.Plane_Linux_cmd(7, 1, mode.value, frequency, ammo)
        return CommandResult.SUCCESS

    def is_laser_hit(self) -> bool:
        """
        Check if laser receiver detected a hit.

        Returns:
            True if hit detected, False otherwise

        Note:
            Requires laser receiver to be enabled first with:
            drone.fire_laser(LaserMode.RECEIVER_ON)
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        return self._server.get_laser_receiving()

    # ==================== Camera Control ====================

    def set_camera_angle(
        self,
        mode: CameraPitchMode,
        angle: int = 0,
    ) -> CommandResult:
        """
        Control main camera pitch angle.

        Args:
            mode: Pitch control mode (UP_ABSOLUTE, DOWN_ABSOLUTE, etc.)
            angle: Target angle in degrees (0-90)

        Returns:
            CommandResult

        Example:
        ```python
        # Look down at 45 degrees
        drone.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 45)

        # Look straight ahead (0 degrees)
        drone.set_camera_angle(CameraPitchMode.UP_ABSOLUTE, 0)

        # Calibrate camera
        drone.set_camera_angle(CameraPitchMode.CALIBRATE)
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        angle = max(0, min(90, angle))
        # Plane_Linux_cmd(8, ...) is camera angle control
        self._server.Plane_Linux_cmd(8, 1, mode.value, angle, 0)
        return CommandResult.SUCCESS

    # ==================== Video Stream Control ====================

    def set_video_stream(self, enabled: bool) -> CommandResult:
        """
        Enable or disable video stream.

        Args:
            enabled: True to enable, False to disable

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        mode = VideoStreamMode.ENABLE if enabled else VideoStreamMode.DISABLE
        # Plane_Linux_cmd(9, ...) is RTP video stream control
        self._server.Plane_Linux_cmd(9, 1, mode.value, 0, 0)
        return CommandResult.SUCCESS

    def flip_video(self) -> CommandResult:
        """
        Flip video stream orientation.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        self._server.single_fly_flip_rtp()
        return CommandResult.SUCCESS

    def set_qr_localization(self, enabled: bool) -> CommandResult:
        """
        Enable or disable QR code localization.

        When enabled, the drone uses QR codes on a mat for absolute position tracking.
        This significantly changes how positioning works:

        Effects when ENABLED:

        - get_position() returns absolute coordinates in the QR mat coordinate system
        - move_to(x, y, z) flies to absolute position (x, y, z) on the QR mat
        - curve_to() uses absolute target coordinates
        - Enables QR-based features: recognize_qr(), track_qr(), QR landing
        - Required for accurate multi-drone formation flight

        Effects when DISABLED (default):

        - Position starts at (0, 0, 0) on takeoff
        - get_position() returns position relative to takeoff point
        - move_to(x, y, z) flies to position relative to takeoff origin
        - Position tracking uses optical flow (less accurate, may drift)

        Args:
            enabled: True to enable, False to disable

        Returns:
            CommandResult

        Example:
        ```python
        # Enable QR localization for absolute positioning
        drone.set_qr_localization(enabled=True)

        # Now move_to uses absolute QR mat coordinates
        drone.move_to(x=200, y=350, z=120)  # Fly to exact position

        # Position reports are now absolute
        pos = drone.get_position()  # Returns absolute QR coordinates
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        mode = QRLocalizationMode.ENABLE if enabled else QRLocalizationMode.DISABLE
        # Plane_Linux_cmd(10, ...) is QR localization control
        self._server.Plane_Linux_cmd(10, 1, mode.value, 0, 0)
        return CommandResult.SUCCESS

    def set_video_resolution(
        self, resolution: VideoResolution | int
    ) -> CommandResult:
        """
        Set video resolution for RTP streaming and recording.

        Controls the video encoder resolution. Lower resolution reduces encoder
        CPU load, which may allow the QR localization camera to run at a higher
        update rate when RTP streaming is active.

        By default, enabling RTP streaming causes QR updates to drop to 5 Hz.
        Using a lower resolution may help mitigate this (needs testing).

        Args:
            resolution: Resolution level - use VideoResolution enum or int:

                - VideoResolution.HIGH (0): 1920x1080 - Best quality
                - VideoResolution.MEDIUM (1): 1280x720 - Balanced
                - VideoResolution.LOW (2): 640x480 or lower - AI/programming mode

        Returns:
            CommandResult

        Example:
        ```python
        # Set low resolution before enabling RTP for object detection
        from pyhulax.core import VideoResolution

        drone.set_video_resolution(VideoResolution.LOW)
        drone.enable_rtp()  # Now streaming at lower resolution
        # ... capture frames for object detection
        ```

        Note:

            - Should be set BEFORE enabling RTP streaming for best results
            - May need to disable/re-enable RTP after changing for effect
            - Firmware handler: HandleMsgSelectRecordResolution @ avmanager
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        result = self._server.set_video_resolution(int(resolution))
        return self._parse_result(result)

    # ==================== System Commands ====================

    def get_firmware_version(self) -> CommandResult:
        """
        Request firmware version from the drone.

        Sends VERSION command (cmd=21) to query the avmanager firmware version.
        The response is received asynchronously via plane_ack message.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_VERSION (cmd=21)
            Version info may be available in telemetry after this call.
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=21 is VERSION
        self._server.Plane_Linux_cmd(21, 1, 0, 0, 0)
        return CommandResult.SUCCESS

    def get_mcu_version(self) -> CommandResult:
        """
        Request STM51 MCU firmware version.

        Sends 51VERSION command (cmd=27) to query the flight controller
        MCU firmware version.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_51VERSION (cmd=27)
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=27 is 51VERSION
        self._server.Plane_Linux_cmd(27, 1, 0, 0, 0)
        return CommandResult.SUCCESS

    def shutdown(self) -> CommandResult:
        """
        Shutdown the drone.

        WARNING:
            This will power off the drone. Use with caution.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_SHUTDOWN_REBOOT (cmd=24) with type=0
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=24 is SHUTDOWN_REBOOT, type=0 for shutdown
        self._server.Plane_Linux_cmd(24, 1, 0, 0, 0)
        return CommandResult.SUCCESS

    def reboot(self) -> CommandResult:
        """
        Reboot the drone.

        WARNING:
            This will restart the drone. Connection will be lost.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_SHUTDOWN_REBOOT (cmd=24) with type=1
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=24 is SHUTDOWN_REBOOT, type=1 for reboot
        self._server.Plane_Linux_cmd(24, 1, 1, 0, 0)
        return CommandResult.SUCCESS

    def get_storage_capacity(self) -> CommandResult:
        """
        Request SD card storage capacity information.

        Sends STORAGE_CAPACITY command (cmd=2) to query available
        and total storage space on the drone's SD card.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_STORAGE_CAPACITY (cmd=2)
            Storage info may be available in telemetry after this call.
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=2 is STORAGE_CAPACITY
        self._server.Plane_Linux_cmd(2, 1, 0, 0, 0)
        return CommandResult.SUCCESS

    def set_anti_flicker(self, hz_50: bool = True) -> CommandResult:
        """
        Set anti-flicker mode for indoor lighting.

        Adjusts camera exposure to reduce flickering under artificial lights.
        Use 50Hz for most of Europe/Asia, 60Hz for Americas.

        Args:
            hz_50: True for 50Hz (default), False for 60Hz

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_RESIST_SCREEN_FLICKER (cmd=23)

            - type=0: 50Hz
            - type=1: 60Hz
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=23 is RESIST_SCREEN_FLICKER, type=0 for 50Hz, type=1 for 60Hz
        flicker_type = 0 if hz_50 else 1
        self._server.Plane_Linux_cmd(23, 1, flicker_type, 0, 0)
        return CommandResult.SUCCESS

    def sync_time(self) -> CommandResult:
        """
        Synchronize drone clock with current time.

        Sends SYNC_TIME command (cmd=25) to update the drone's
        internal clock.

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_SYNC_TIME (cmd=25)
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        # cmd=25 is SYNC_TIME
        self._server.Plane_Linux_cmd(25, 1, 0, 0, 0)
        return CommandResult.SUCCESS

    # ==================== WiFi Configuration ====================

    def set_wifi_mode(self, mode: "WiFiMode", channel_id: int = 0) -> CommandResult:
        """
        Set WiFi configuration mode.

        Low-level method to configure WiFi settings. For common operations,
        prefer using the convenience methods (set_wifi_band, set_wifi_power, etc.).

        Args:
            mode: WiFiMode enum value
            channel_id: Channel ID for manual channel mode (WiFiMode.CHANNEL_MANUAL)

        Returns:
            CommandResult

        Note:
            Uses MAV_PLANE_CMD_WIFI_MODE (cmd=4)
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        result = self._server.set_wifi_mode(int(mode), channel_id)
        return self._parse_result(result)

    def set_wifi_band(self, band_5ghz: bool = False) -> CommandResult:
        """
        Switch WiFi band between 2.4GHz and 5GHz.

        Args:
            band_5ghz: True for 5GHz, False for 2.4GHz (default)

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        mode = WiFiMode.BAND_5GHZ if band_5ghz else WiFiMode.BAND_2_4GHZ
        result = self._server.set_wifi_mode(int(mode))
        return self._parse_result(result)

    def set_wifi_power(self, high: bool = True) -> CommandResult:
        """
        Set WiFi transmission power.

        Args:
            high: True for high power (default), False for low power

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        mode = WiFiMode.POWER_HIGH if high else WiFiMode.POWER_LOW
        result = self._server.set_wifi_mode(int(mode))
        return self._parse_result(result)

    def set_wifi_broadcast(self, enabled: bool = True) -> CommandResult:
        """
        Enable or disable WiFi broadcast.

        Args:
            enabled: True to enable broadcast (default), False to disable

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        mode = WiFiMode.BROADCAST_ON if enabled else WiFiMode.BROADCAST_OFF
        result = self._server.set_wifi_mode(int(mode))
        return self._parse_result(result)

    def set_wifi_channel(self, manual: bool = False, channel_id: int = 0) -> CommandResult:
        """
        Set WiFi channel mode.

        Args:
            manual: True for manual channel selection, False for auto (default)
            channel_id: Channel ID for manual mode

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        mode = WiFiMode.CHANNEL_MANUAL if manual else WiFiMode.CHANNEL_AUTO
        result = self._server.set_wifi_mode(int(mode), channel_id)
        return self._parse_result(result)

    def set_wifi_ap_mode(self) -> CommandResult:
        """
        Switch WiFi to Access Point mode.

        Returns:
            CommandResult
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        from pyhulax.core import WiFiMode
        result = self._server.set_wifi_mode(int(WiFiMode.AP_MODE))
        return self._parse_result(result)

    # ==================== Video Streaming ====================

    def start_video_stream(
        self,
        display: bool = True,
        web_server: bool = False,
        web_port: int | None = None,
    ) -> VideoStream:
        """
        Start video stream with optional display and web server.

        Enables the drone's RTP video stream and returns a VideoStream
        instance for frame processing and object detection.

        Args:
            display: Show OpenCV window with video
            web_server: Start web server for browser viewing
            web_port: Web server port. Defaults to config network web port.

        Returns:
            VideoStream instance for adding detection callbacks

        Example:
        ```python
        # Basic display
        stream = drone.start_video_stream()

        # With object detection
        def detect(frame):
            frame.detections = my_model.detect(frame.image)
            return frame

        stream = drone.start_video_stream()
        stream.add_callback(detect)

        # With web streaming
        stream = drone.start_video_stream(web_server=True)
        print("Open http://localhost:5000 in browser")
        ```
        """
        from pyhulax.video import VideoStream, VideoDisplay

        if not self._connected:
            raise NotReady("Not connected to drone")

        # Enable RTP stream on drone
        self.set_video_stream(True)

        # Get drone IP and plane_id
        drone_ip = getattr(self._server, "_server_ip", self._config.network.drone_ip)
        drone_id = self.get_drone_id() or 1

        # Create stream with correct drone_id for port calculation
        stream = VideoStream(
            drone_ip=drone_ip,
            drone_id=drone_id,
            config=self._config,
        )

        # Add display if requested
        if display:
            video_display = VideoDisplay(
                window_name="Drone Video",
                show_fps=True,
                show_detections=True,
            )
            stream.add_callback(video_display)

        # Add web server if requested
        if web_server:
            try:
                from pyhulax.video import WebStreamServer
            except ImportError:
                logger.warning("Flask not installed. Web streaming unavailable.")
            else:
                server = WebStreamServer(
                    stream,
                    port=web_port or self._config.network.web_port,
                )
                server.start()
                logger.info(f"Web video stream at {server.url}")

        # Start stream
        stream.start()

        return stream

    def create_video_stream(self) -> VideoStream:
        """
        Create a video stream without starting it.

        Use this when you want full control over the stream configuration
        and callback pipeline before starting.

        Returns:
            VideoStream instance (not started)

        Example:
        ```python
        stream = drone.create_video_stream()

        # Add callbacks
        stream.add_callback(my_detector)
        stream.add_callback(my_display)

        # Enable stream on drone and start
        drone.set_video_stream(True)
        stream.start()
        ```
        """
        from pyhulax.video import VideoStream

        if not self._connected:
            raise NotReady("Not connected to drone")

        drone_ip = getattr(self._server, "_server_ip", self._config.network.drone_ip)
        drone_id = self.get_drone_id() or 1
        return VideoStream(drone_ip=drone_ip, drone_id=drone_id, config=self._config)

    # ==================== Media Management ====================

    def _get_media_dir(self, media_type: MediaType) -> Path:
        """Get the local directory for storing media of given type."""
        if media_type == MediaType.PHOTO:
            return self._photo_dir
        elif media_type == MediaType.VIDEO:
            return self._video_dir
        else:
            return self._log_dir

    def _list_media(self, media_type: MediaType, page: int = 0) -> list[MediaFile]:
        """
        List media files of given type from drone.

        Args:
            media_type: Type of media to list (PHOTO, VIDEO, LOG)
            page: Page number for pagination (0-indexed)

        Returns:
            List of MediaFile objects sorted by date (newest first)
        """
        url = (
            f"http://{self._drone_ip}:{self._config.network.http_port}"
            f"/http.cgi?media_type={media_type.value}&page={page}"
        )
        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status()
            data = response.json()
            snaplist = data.get("snaplist", [])
            return [MediaFile.from_snap_info(item, media_type) for item in snaplist]
        except requests.RequestException as e:
            if self._log:
                self._log.error(f"Failed to list {media_type.name}: {e}")
            return []

    def list_photos(self, page: int = 0) -> list[MediaFile]:
        """
        List photos stored on drone.

        Args:
            page: Page number for pagination (0-indexed)

        Returns:
            List of MediaFile objects sorted by date (newest first)

        Example:
        ```python
        photos = drone.list_photos()
        for photo in photos:
            print(f"{photo.name} - {photo.size} - {photo.date}")
        ```
        """
        return self._list_media(MediaType.PHOTO, page)

    def list_videos(self, page: int = 0) -> list[MediaFile]:
        """
        List videos stored on drone.

        Args:
            page: Page number for pagination (0-indexed)

        Returns:
            List of MediaFile objects sorted by date (newest first)
        """
        return self._list_media(MediaType.VIDEO, page)

    def list_logs(self, page: int = 0) -> list[MediaFile]:
        """
        List log files stored on drone.

        Args:
            page: Page number for pagination (0-indexed)

        Returns:
            List of MediaFile objects sorted by date (newest first)
        """
        return self._list_media(MediaType.LOG, page)

    def _download_media(
        self,
        media_file: MediaFile,
        save_dir: Path | None = None,
    ) -> Path | None:
        """
        Download a media file from drone.

        Args:
            media_file: MediaFile object to download
            save_dir: Directory to save to (uses default based on media type if None)

        Returns:
            Path to downloaded file, or None if download failed
        """
        if save_dir is None:
            save_dir = self._get_media_dir(media_file.media_type)

        # Create directory if needed
        save_dir.mkdir(parents=True, exist_ok=True)

        url = media_file.get_download_url(self._drone_ip, self._config.network.http_port)
        save_path = save_dir / media_file.name

        try:
            response = requests.get(url, stream=True, timeout=30)
            response.raise_for_status()

            with open(save_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)

            if self._log:
                self._log.info(f"Downloaded {media_file.name} to {save_path}")
            return save_path

        except requests.RequestException as e:
            if self._log:
                self._log.error(f"Failed to download {media_file.name}: {e}")
            return None

    def _download_media_to_path(
        self,
        media_file: MediaFile,
        save_path: Path | str | None = None,
    ) -> Path | None:
        """Download media to its default directory, a custom directory, or an exact file path."""
        if save_path is None:
            return self._download_media(media_file)

        requested_path = Path(save_path)

        if requested_path.exists() and requested_path.is_dir():
            return self._download_media(media_file, save_dir=requested_path)

        if requested_path.suffix:
            downloaded = self._download_media(media_file, save_dir=requested_path.parent)
            if downloaded is None:
                return None

            requested_path.parent.mkdir(parents=True, exist_ok=True)
            if downloaded != requested_path:
                shutil.move(str(downloaded), str(requested_path))
            return requested_path

        return self._download_media(media_file, save_dir=requested_path)

    def download_photo(
        self,
        photo: MediaFile | str,
        save_dir: Path | str | None = None,
    ) -> Path | None:
        """
        Download a photo from drone.

        Args:
            photo: MediaFile object or filename string
            save_dir: Directory to save to (default: media/photos/)

        Returns:
            Path to downloaded file, or None if download failed

        Example:
        ```python
        # Download by MediaFile
        photos = drone.list_photos()
        path = drone.download_photo(photos[0])

        # Download by filename
        path = drone.download_photo("20251111_102056_1.jpg")
        ```
        """
        if isinstance(photo, str):
            photos = self.list_photos()
            matching = [p for p in photos if p.name == photo]
            if not matching:
                if self._log:
                    self._log.error(f"Photo not found: {photo}")
                return None
            media_file: MediaFile = matching[0]
        else:
            media_file = photo

        save_path = Path(save_dir) if save_dir else None
        return self._download_media(media_file, save_path)

    def download_video(
        self,
        video: MediaFile | str,
        save_dir: Path | str | None = None,
    ) -> Path | None:
        """
        Download a video from drone.

        Args:
            video: MediaFile object or filename string
            save_dir: Directory to save to (default: media/videos/)

        Returns:
            Path to downloaded file, or None if download failed
        """
        if isinstance(video, str):
            videos = self.list_videos()
            matching = [v for v in videos if v.name == video]
            if not matching:
                if self._log:
                    self._log.error(f"Video not found: {video}")
                return None
            media_file: MediaFile =  matching[0]
        else:
            media_file = video

        save_path = Path(save_dir) if save_dir else None
        return self._download_media(media_file, save_path)

    def download_log(
        self,
        log: MediaFile | str,
        save_dir: Path | str | None = None,
    ) -> Path | None:
        """
        Download a log file from drone.

        Args:
            log: MediaFile object or filename string
            save_dir: Directory to save to (default: media/logs/)

        Returns:
            Path to downloaded file, or None if download failed
        """
        if isinstance(log, str):
            logs = self.list_logs()
            matching = [l for l in logs if l.name == log]
            if not matching:
                if self._log:
                    self._log.error(f"Log not found: {log}")
                return None
            media_file: MediaFile = matching[0]
        else:
            media_file = log

        save_path = Path(save_dir) if save_dir else None
        return self._download_media(media_file, save_path)

    def download_all_photos(self, save_dir: Path | str | None = None) -> list[Path]:
        """
        Download all photos from drone.

        Args:
            save_dir: Directory to save to (default: media/photos/)

        Returns:
            List of paths to downloaded files
        """
        photos = self.list_photos()
        downloaded = []
        for photo in photos:
            path = self.download_photo(photo, save_dir)
            if path:
                downloaded.append(path)
        return downloaded

    def download_all_videos(self, save_dir: Path | str | None = None) -> list[Path]:
        """
        Download all videos from drone.

        Args:
            save_dir: Directory to save to (default: media/videos/)

        Returns:
            List of paths to downloaded files
        """
        videos = self.list_videos()
        downloaded = []
        for video in videos:
            path = self.download_video(video, save_dir)
            if path:
                downloaded.append(path)
        return downloaded

    def download_all_logs(self, save_dir: Path | str | None = None) -> list[Path]:
        """
        Download all log files from drone.

        Args:
            save_dir: Directory to save to (default: media/logs/)

        Returns:
            List of paths to downloaded files
        """
        logs = self.list_logs()
        downloaded = []
        for log in logs:
            path = self.download_log(log, save_dir)
            if path:
                downloaded.append(path)
        return downloaded

    def _delete_media(self, media_file: MediaFile) -> bool:
        """
        Delete a media file from drone.

        Args:
            media_file: MediaFile object to delete

        Returns:
            True if deletion successful
        """
        url = (
            f"http://{self._drone_ip}:{self._config.network.http_port}"
            f"/http.cgi?del_type={media_file.media_type.value}&filename={media_file.name}"
        )
        try:
            response = requests.get(url, timeout=5)
            if self._log:
                self._log.info(f"Deleted {media_file.name} from drone")
            return response.ok
        except requests.RequestException as e:
            if self._log:
                self._log.error(f"Failed to delete {media_file.name}: {e}")
            return False

    def delete_photo(self, photo: MediaFile | str) -> bool:
        """
        Delete a photo from drone.

        Args:
            photo: MediaFile object or filename string

        Returns:
            True if deletion successful

        Example:
        ```python
        # Delete by MediaFile
        photos = drone.list_photos()
        drone.delete_photo(photos[0])

        # Delete by filename
        drone.delete_photo("20251111_102056_1.jpg")
        ```
        """
        if isinstance(photo, str):
            photo: MediaFile = MediaFile(
                name=photo,
                path=f"/sdcard/picture/{photo}",
                date="",
                size="",
                media_type=MediaType.PHOTO,
            )
        return self._delete_media(photo)

    def delete_video(self, video: MediaFile | str) -> bool:
        """
        Delete a video from drone.

        Args:
            video: MediaFile object or filename string

        Returns:
            True if deletion successful
        """
        if isinstance(video, str):
            video: MediaFile = MediaFile(
                name=video,
                path=f"/sdcard/video/{video}",
                date="",
                size="",
                media_type=MediaType.VIDEO,
            )
        return self._delete_media(video)

    def delete_log(self, log: MediaFile | str) -> bool:
        """
        Delete a log file from drone.

        Args:
            log: MediaFile object or filename string

        Returns:
            True if deletion successful
        """
        if isinstance(log, str):
            log = MediaFile(
                name=log,
                path=f"/sdcard/log/{log}",
                date="",
                size="",
                media_type=MediaType.LOG,
            )
        return self._delete_media(log)

    def delete_all_photos(self) -> tuple[int, int]:
        """
        Delete all photos from drone SD card.

        Returns:
            Tuple of (deleted_count, failed_count)

        Example:
        ```python
        deleted, failed = drone.delete_all_photos()
        print(f"Deleted {deleted} photos, {failed} failed")
        ```
        """
        photos = self.list_photos()
        deleted = 0
        failed = 0
        for photo in photos:
            if self.delete_photo(photo):
                deleted += 1
            else:
                failed += 1
        if self._log:
            self._log.info(f"Bulk delete photos: {deleted} deleted, {failed} failed")
        return deleted, failed

    def delete_all_videos(self) -> tuple[int, int]:
        """
        Delete all videos from drone SD card.

        Returns:
            Tuple of (deleted_count, failed_count)

        Example:
        ```python
        deleted, failed = drone.delete_all_videos()
        print(f"Deleted {deleted} videos, {failed} failed")
        ```
        """
        videos = self.list_videos()
        deleted = 0
        failed = 0
        for video in videos:
            if self.delete_video(video):
                deleted += 1
            else:
                failed += 1
        if self._log:
            self._log.info(f"Bulk delete videos: {deleted} deleted, {failed} failed")
        return deleted, failed

    def delete_all_logs(self) -> tuple[int, int]:
        """
        Delete all log files from drone SD card.

        Returns:
            Tuple of (deleted_count, failed_count)

        Example:
        ```python
        deleted, failed = drone.delete_all_logs()
        print(f"Deleted {deleted} logs, {failed} failed")
        ```
        """
        logs = self.list_logs()
        deleted = 0
        failed = 0
        for log in logs:
            if self.delete_log(log):
                deleted += 1
            else:
                failed += 1
        if self._log:
            self._log.info(f"Bulk delete logs: {deleted} deleted, {failed} failed")
        return deleted, failed

    def delete_all_media(self) -> dict[str, tuple[int, int]]:
        """
        Delete all media (photos, videos, logs) from drone SD card.

        Returns:
            Dict with keys 'photos', 'videos', 'logs', each containing
            (deleted_count, failed_count) tuple

        Example:
        ```python
        results = drone.delete_all_media()
        for media_type, (deleted, failed) in results.items():
            print(f"{media_type}: {deleted} deleted, {failed} failed")
        ```
        """
        return {
            "photos": self.delete_all_photos(),
            "videos": self.delete_all_videos(),
            "logs": self.delete_all_logs(),
        }

    def get_photo_url(self, photo: MediaFile | str) -> str:
        """
        Get direct HTTP URL for a photo.

        Args:
            photo: MediaFile object or filename string

        Returns:
            HTTP URL for downloading the photo

        Example:
        ```python
        url = drone.get_photo_url("20251111_102056_1.jpg")
        # http://192.168.100.1:12346/picture/20251111_102056_1.jpg
        ```
        """
        if isinstance(photo, str):
            return (
                f"http://{self._drone_ip}:{self._config.network.http_port}"
                f"/picture/{photo}"
            )
        return photo.get_download_url(self._drone_ip, self._config.network.http_port)

    def get_video_url(self, video: MediaFile | str) -> str:
        """
        Get direct HTTP URL for a video.

        Args:
            video: MediaFile object or filename string

        Returns:
            HTTP URL for downloading the video
        """
        if isinstance(video, str):
            return (
                f"http://{self._drone_ip}:{self._config.network.http_port}"
                f"/video/{video}"
            )
        return video.get_download_url(self._drone_ip, self._config.network.http_port)

    # ==================== Manual Control ====================

    def send_manual_control(
        self,
        forward: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        rotate: float = 0.0,
    ) -> bool:
        """
        Send a single manual control frame for joystick-style flight.

        This sends a MANUAL_CONTROL MAVLink message via UDP, allowing
        simultaneous position and yaw control. Call this at ~20Hz for
        smooth control (like a joystick).

        Unlike move() and rotate() which are blocking commands that complete
        a specific movement, this provides continuous real-time control.

        Args:
            forward: Forward/back input, -1.0 to +1.0. Positive = forward.
            right: Left/right input, -1.0 to +1.0. Positive = right.
            up: Up/down input, -1.0 to +1.0. Positive = up.
            rotate: Rotation input, -1.0 to +1.0. Positive = CCW (left turn).

        Returns:
            True if message was sent successfully.

        Example:
        ```python
        # Move forward while rotating left
        for _ in range(40):  # 2 seconds at 20Hz
            drone.send_manual_control(forward=0.5, rotate=0.3)
            time.sleep(0.05)
        drone.send_manual_control()  # Stop
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        # Convert -1.0 to +1.0 range to -1000 to +1000
        x = int(max(-1.0, min(1.0, forward)) * 1000)
        y = int(max(-1.0, min(1.0, right)) * 1000)
        z = int(max(-1.0, min(1.0, up)) * 1000)
        r = int(max(-1.0, min(1.0, rotate)) * 1000)

        return self._server.send_manual_control(x, y, z, r)

    def manual_fly(
        self,
        duration_sec: float,
        forward: float = 0.0,
        right: float = 0.0,
        up: float = 0.0,
        rotate: float = 0.0,
        rate_hz: int = 20,
        on_frame: ManualFlyFrameCallback | None = None,
    ) -> bool:
        """
        Fly with manual control inputs for a specified duration.

        This provides simultaneous position and yaw movement - unlike
        the standard move() and rotate() commands which must be called
        separately.

        Velocity is controlled by the manual velocity level set via
        set_velocity_level(). Default levels:

        - Level 1 (SLOW): 1.0 m/s horizontal, 0.5 m/s vertical
        - Level 2 (MEDIUM): 2.0 m/s horizontal, 0.8-1.0 m/s vertical
        - Level 3 (FAST): 3.0 m/s horizontal, 1.0-1.2 m/s vertical

        Args:
            duration_sec: How long to fly with these inputs (seconds).
            forward: Forward/back input, -1.0 to +1.0. Positive = forward.
            right: Left/right input, -1.0 to +1.0. Positive = right.
            up: Up/down input, -1.0 to +1.0. Positive = up.
            rotate: Rotation input, -1.0 to +1.0. Positive = CCW (left turn).
            rate_hz: Control loop rate (default 20 Hz = 50ms interval).
            on_frame: Optional callback(x, y, z, r, frame_index, success) called after each frame.

        Returns:
            True if all messages were sent successfully.

        Example:
        ```python
        # Arc movement: forward + rotation for 3 seconds
        drone.manual_fly(3.0, forward=0.6, rotate=0.4)

        # Diagonal strafe while climbing
        drone.manual_fly(2.0, forward=0.5, right=0.5, up=0.3)

        # Orbit-like: rotate while moving right
        drone.manual_fly(5.0, right=0.3, rotate=0.5)

        # With frame logging callback
        def log_frame(x, y, z, r, idx, ok):
            print(f"Frame {idx}: x={x}, y={y}, z={z}, r={r}, ok={ok}")
        drone.manual_fly(2.0, forward=0.5, on_frame=log_frame)
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        return self._server.manual_fly(
            duration_sec=duration_sec,
            forward=forward,
            right=right,
            up=up,
            rotate=rotate,
            rate_hz=rate_hz,
            on_frame=on_frame,
        )

    def send_app_heartbeat(self, user_mode: int = 1) -> bool:
        """
        Send APP_HEARTBEAT message to set the app control mode.

        The drone requires periodic heartbeats to accept certain control modes.
        This is called automatically by manual_fly(), but can be called
        directly for custom control loops using manual_control_frame().

        User mode values:
            0 = Other
            1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
            2 = Program (autonomous flight mode)
            3 = Battle
            4 = Formation

        Args:
            user_mode: App mode (0-4). Default 1 for manual flight.

        Returns:
            True if message was sent successfully.

        Example:
        ```python
        # Custom control loop with manual heartbeat management
        drone.send_app_heartbeat(user_mode=1)  # Enter Aerial mode
        for _ in range(100):
            drone.manual_control_frame(forward=0.5)
            if iteration % 20 == 0:  # Every ~1 second at 20Hz
                drone.send_app_heartbeat(user_mode=1)
            time.sleep(0.05)
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        return self._server.send_app_heartbeat(user_mode)

    def set_app_mode(self, mode: int) -> None:
        """
        Set the app mode for background heartbeat messages.

        This changes the user_mode sent in periodic APP_HEARTBEAT messages.
        The background heartbeat thread sends this mode every second.

        IMPORTANT:
            Call this BEFORE takeoff when you want to use manual control.

            The drone needs to receive Aerial mode (1) heartbeats to accept MANUAL_CONTROL messages.

        Mode values:

        - 0 = Other
        - 1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
        - 2 = Program (autonomous flight mode) - default
        - 3 = Battle
        - 4 = Formation

        Args:
            mode: App mode (0-4).

        Example:
        ```python
        drone.connect("192.168.100.1")
        drone.set_app_mode(1)  # Switch to Aerial mode for manual control
        drone.takeoff()
        drone.manual_fly(duration_sec=5, up=0.5)
        ```
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        self._server.set_app_mode(mode)

    def get_app_mode(self) -> int:
        """
        Get the current app mode for background heartbeat messages.

        Returns:
            Current app mode (0-4).
        """
        if not self._connected:
            raise NotReady("Not connected to drone")
        return self._server.get_app_mode()

    def stop_manual_control(self) -> bool:
        """
        Send zero inputs to stop manual movement.

        Call this after manual control to ensure the drone stops
        any ongoing movement from manual inputs.

        Returns:
            True if message was sent successfully.
        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        return self._server.stop_manual_control()

    # ==================== Flight Controller ====================

    def create_flight_controller(
        self, config: FlightControllerConfig | None = None
    ) -> ManualFlightController:
        """
        Create a closed-loop PD flight controller.

        The controller uses MANUAL_CONTROL messages to provide continuous
        joystick-style inputs, enabling **simultaneous position (xyz) and
        yaw control** - something the blocking move_to() + rotate() commands
        cannot do.

        The controller runs at 20Hz (configurable) and uses PD control to
        smoothly reach target positions while maintaining heading.

        Args:
            config: Controller configuration. Uses this API instance's runtime
                    config defaults if None. See FlightControllerConfig for
                    tunable parameters.

        Returns:
            ManualFlightController instance ready for use.

        Raises:
            NotReady: If not connected to drone.

        Example:
        ```python
        with DroneAPI() as drone:
            drone.connect("192.168.100.1")
            drone.takeoff()

            # Create controller with custom gains
            ctrl = drone.create_flight_controller()
            ctrl.configure(kp_xy=2.5, position_tolerance_cm=3.0)

            # Fly to target with simultaneous yaw
            result = ctrl.fly_to(x=100, y=200, z=120, yaw=90)
            print(f"Arrived: {result.success}, error: {result.error_position_cm}cm")

            # Manual control loop
            ctrl.set_target(x=50, y=50, z=100, yaw=180)
            while not ctrl.has_converged():
                ctrl.update()
                time.sleep(0.05)
            ctrl.stop()

            drone.land()
        ```

        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        controller_config = (
            config
            if config is not None
            else FlightControllerConfig.from_drone_config(self._config)
        )
        return ManualFlightController(self, controller_config)

    # ==================== Internal Helpers ====================

    def _check_ready(self, check_flying: bool = True) -> None:
        """
        Verify drone is ready for commands.

        Args:
            check_flying: If True, also check battery for flight operations

        Raises:
            NotReady: If not connected or no heartbeat
            LowBattery: If battery below threshold (when check_flying=True)
        """
        if not self._connected:
            raise NotReady("Not connected to drone")

        heartbeat = self._server.get_plane_data("heartbeat")
        if heartbeat is None:
            raise NotReady("No heartbeat from drone")

        if check_flying:
            flight_data = self._server.get_plane_data("flight_data")
            if flight_data and flight_data.battery_volumn < self._battery_threshold:
                raise LowBattery(flight_data.battery_volumn, self._battery_threshold)

    @staticmethod
    def _parse_result(result) -> CommandResult:
        """
        Parse command result into CommandResult enum.

        Args:
            result: Raw result from Controlserver method

        Returns:
            CommandResult enum value
        """
        if result is None:
            return CommandResult.TIMEOUT
        if isinstance(result, bool):
            return CommandResult.SUCCESS if result else CommandResult.FAILED_241
        if isinstance(result, int):
            try:
                return CommandResult(result)
            except ValueError:
                return CommandResult.UNKNOWN
        return CommandResult.SUCCESS


__all__ = [
    "DroneAPI",
    "DroneConfig",
    "NetworkConfig",
    "ProtocolConfig",
    "DronePhysicsConfig",
    "FlightConfig",
    "ControllerDefaultsConfig",
    "VideoConfig",
    "TimeoutConfig",
    "BatteryConfig",
    "MediaConfig",
    "FlightControllerConfig",
]
