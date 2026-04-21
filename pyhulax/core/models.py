"""
Pydantic models for Drone API inputs and outputs.

All models use Pydantic v2 for runtime validation and serialization.
"""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field, ConfigDict

from .types import LEDMode, DroneStatus, MediaType


IGNORED_MODEL_TYPES = (type(lambda: None), classmethod, staticmethod, property)


class SDKModel(BaseModel):
    """Base Pydantic model compatible with both source and Cython builds."""

    model_config = ConfigDict(ignored_types=IGNORED_MODEL_TYPES)


class FrozenSDKModel(SDKModel):
    """Frozen variant of the shared SDK model base."""

    model_config = ConfigDict(frozen=True, ignored_types=IGNORED_MODEL_TYPES)


class Vector3(FrozenSDKModel):
    """3D vector for position, velocity, or acceleration.

    Units depend on context:

    - Position: centimeters (cm)
    - Velocity: cm/s
    - Acceleration: cm/s^2
    """
    x: float
    y: float
    z: float

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(x=self.x * scalar, y=self.y * scalar, z=self.z * scalar)

    def magnitude(self) -> float:
        """Calculate vector magnitude."""
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5


class Orientation(FrozenSDKModel):
    """Orientation angles in degrees.

    Yaw: rotation around vertical axis (0-360)
    Pitch: nose up/down (-90 to 90)
    Roll: wing tilt (-180 to 180)
    """
    yaw: float
    pitch: float
    roll: float


class LEDConfig(FrozenSDKModel):
    """LED configuration for flight commands.

    Replaces the untyped dict `{'r': 0, 'g': 0, 'b': 0, 'mode': 1}` pattern.

    Usage:
    ```python
    # Using predefined colors
    from pyhulax.core import LEDColor
    led = LEDColor.RED
    led = LEDColor.BLUE.with_mode(LEDMode.BLINK)

    # Using custom RGB
    led = LEDConfig.rgb(128, 64, 255)
    led = LEDConfig(r=255, g=128, b=0, mode=LEDMode.BREATHING)
    ```
    """
    r: int = Field(default=0, ge=0, le=255, description="Red channel (0-255)")
    g: int = Field(default=0, ge=0, le=255, description="Green channel (0-255)")
    b: int = Field(default=0, ge=0, le=255, description="Blue channel (0-255)")
    mode: LEDMode = Field(default=LEDMode.CONSTANT, description="LED mode")

    def to_param4(self) -> int:
        """Convert to MAVLink param4 format for formation_cmd_encode().

        Bit layout: `[mode:8][b:8][g:8][r:8]`
        """
        return self.r | (self.g << 8) | (self.b << 16) | (self.mode << 24)

    def with_mode(self, mode: LEDMode) -> "LEDConfig":
        """Return new LEDConfig with different mode, keeping same color."""
        return LEDConfig(r=self.r, g=self.g, b=self.b, mode=mode)

    # ==================== Factory Methods ====================

    @classmethod
    def off(cls) -> "LEDConfig":
        """Create LED-off configuration."""
        return cls(r=0, g=0, b=0, mode=LEDMode.OFF)

    @classmethod
    def rgb(cls, r: int, g: int, b: int, mode: LEDMode = LEDMode.CONSTANT) -> "LEDConfig":
        """Create custom RGB LED configuration."""
        return cls(r=r, g=g, b=b, mode=mode)


class LEDColor:
    """Predefined LED colors for convenience.

    Usage:
    ```python
    from pyhulax.core import LEDColor, LEDMode

    # Use directly
    drone.move(Direction.FORWARD, 100, led=LEDColor.RED)

    # With different mode
    drone.takeoff(led=LEDColor.GREEN.with_mode(LEDMode.BLINK))

    # Custom RGB still available
    drone.land(led=LEDConfig.rgb(128, 64, 255))
    ```
    """
    # Primary colors
    RED = LEDConfig(r=255, g=0, b=0)
    GREEN = LEDConfig(r=0, g=255, b=0)
    BLUE = LEDConfig(r=0, g=0, b=255)

    # Secondary colors
    YELLOW = LEDConfig(r=255, g=255, b=0)
    CYAN = LEDConfig(r=0, g=255, b=255)
    MAGENTA = LEDConfig(r=255, g=0, b=255)

    # Other common colors
    WHITE = LEDConfig(r=255, g=255, b=255)
    ORANGE = LEDConfig(r=255, g=128, b=0)
    PURPLE = LEDConfig(r=128, g=0, b=255)
    PINK = LEDConfig(r=255, g=105, b=180)
    LIME = LEDConfig(r=50, g=255, b=50)
    SKY_BLUE = LEDConfig(r=135, g=206, b=235)
    WARM_WHITE = LEDConfig(r=255, g=244, b=229)
    GOLD = LEDConfig(r=255, g=215, b=0)
    CORAL = LEDConfig(r=255, g=127, b=80)
    TURQUOISE = LEDConfig(r=64, g=224, b=208)

    # Off
    OFF = LEDConfig(r=0, g=0, b=0, mode=LEDMode.OFF)


class FlightData(SDKModel):
    """Complete flight telemetry snapshot.

    Parsed from MAVLink REPORT_FLIGHT_DATA (msg ID 206).
    """
    position: Vector3 = Field(description="Position in cm")
    velocity: Vector3 = Field(description="Velocity in cm/s")
    acceleration: Vector3 = Field(description="Acceleration in cm/s^2")
    orientation: Orientation = Field(description="Orientation in degrees")
    altitude_tof: float = Field(description="ToF sensor altitude in cm")
    altitude_baro: float = Field(default=0.0, description="Barometer altitude in m")
    battery_percent: int = Field(ge=0, le=100, description="Battery percentage")
    barrier: int = Field(default=0, description="Obstacle detection bitmask")
    timestamp: datetime = Field(default_factory=datetime.now)

    @classmethod
    def from_mavlink(cls, msg) -> "FlightData":
        """Create FlightData from MAVLink report_flight_data message."""
        return cls(
            position=Vector3(x=float(msg.x), y=float(msg.y), z=float(msg.z)),
            velocity=Vector3(x=float(msg.vel_x), y=float(msg.vel_y), z=float(msg.vel_z)),
            acceleration=Vector3(x=float(msg.accx), y=float(msg.accy), z=float(msg.accz)),
            orientation=Orientation(
                yaw=msg.yaw / 100.0,
                pitch=msg.pitch / 100.0,
                roll=msg.roll / 100.0
            ),
            altitude_tof=float(msg.distance),
            altitude_baro=float(msg.baro_alt),
            battery_percent=int(msg.battery_volumn),
            barrier=int(msg.barrier),
        )


class Obstacles(FrozenSDKModel):
    """Obstacle detection state from barrier sensors.

    Parsed from the barrier bitmask in REPORT_FLIGHT_DATA.
    """
    forward: bool = False
    back: bool = False
    left: bool = False
    right: bool = False
    down: bool = False

    @classmethod
    def from_bitmask(cls, barrier: int) -> "Obstacles":
        """Create Obstacles from barrier bitmask.

        Bit mapping:

        - Bit 0: forward
        - Bit 1: back
        - Bit 2: left
        - Bit 3: right
        - Bit 4: down
        """
        return cls(
            forward=(barrier & 1) == 1,
            back=(barrier & 2) == 2,
            left=(barrier & 4) == 4,
            right=(barrier & 8) == 8,
            down=(barrier & 16) == 16,
        )

    def any_obstacle(self) -> bool:
        """Check if any obstacle is detected."""
        return self.forward or self.back or self.left or self.right or self.down


class DroneState(SDKModel):
    """Complete drone state snapshot.

    Combines connection status, telemetry, and obstacle detection.
    """
    status: DroneStatus = Field(description="Current drone status")
    flight_data: Optional[FlightData] = Field(default=None, description="Latest telemetry")
    obstacles: Obstacles = Field(default_factory=Obstacles, description="Obstacle detection")
    drone_id: Optional[int] = Field(default=None, description="Drone ID")
    is_connected: bool = Field(default=False, description="Connection status")
    last_heartbeat: Optional[datetime] = Field(default=None, description="Last heartbeat time")


class AIResult(SDKModel):
    """Result from AI/vision recognition operations.

    Used for digit/arrow recognition, QR code detection, etc.
    """
    success: bool = Field(description="Whether recognition succeeded")
    target_type: int = Field(default=0, description="Recognized target type/value")
    position: Optional[Vector3] = Field(default=None, description="Target position if found")
    angle: Optional[float] = Field(default=None, description="Target angle in degrees")
    qr_id: Optional[int] = Field(default=None, description="QR code ID if applicable")

    @classmethod
    def from_dict(cls, data: dict) -> "AIResult":
        """Create AIResult from legacy dict response."""
        position = None
        if data.get("result"):
            position = Vector3(
                x=float(data.get("x", 0)),
                y=float(data.get("y", 0)),
                z=float(data.get("z", 0))
            )
        return cls(
            success=bool(data.get("result", False)),
            target_type=int(data.get("type", 0)),
            position=position,
            angle=float(data.get("angle", 0)) if data.get("angle") else None,
            qr_id=int(data.get("qr_id")) if data.get("qr_id") else None,
        )


class ColorResult(FrozenSDKModel):
    """Result from color recognition.

    Returns the dominant color detected by the camera.
    """
    success: bool = Field(description="Whether color detection succeeded")
    r: int = Field(default=0, ge=0, le=255, description="Red channel")
    g: int = Field(default=0, ge=0, le=255, description="Green channel")
    b: int = Field(default=0, ge=0, le=255, description="Blue channel")

    @classmethod
    def from_response(cls, r: int, g: int, b: int, state: int) -> "ColorResult":
        """Create ColorResult from legacy response values."""
        return cls(
            success=state == 1,
            r=r,
            g=g,
            b=b,
        )


class SystemStats(SDKModel):
    """Extended system statistics from REPORT_STATS (msg ID 207).

    Contains comprehensive drone status beyond basic telemetry.
    """
    drone_id: int = Field(description="Drone ID")
    firmware_version: int = Field(default=0, description="Firmware version (raw 2-byte value)")
    system_version: int = Field(default=0, description="System version (raw 2-byte value)")
    flight_time: int = Field(default=0, description="Total flight time in ms")
    utc_timestamp: int = Field(default=0, description="UTC timestamp")
    gps_lat: Optional[float] = Field(default=None, description="GPS latitude")
    gps_lon: Optional[float] = Field(default=None, description="GPS longitude")
    sensors_present: int = Field(default=0, description="Sensor presence bitmask")
    sensors_health: int = Field(default=0, description="Sensor health bitmask")
    temperature: int = Field(default=0, description="Internal temperature in C")
    drone_status: int = Field(default=0, description="Drone status flags")
    block_status: int = Field(default=0, description="Block/safety status")
    rgb_status: int = Field(default=0, description="RGB LED status")
    battery_percent: int = Field(default=0, description="Battery percentage")

    @staticmethod
    def parse_version(value: int) -> str:
        """Parse 2-byte version value to dotted string.

        The version is encoded as a 4-digit decimal where each digit
        represents a version component (major.minor.patch.revision).

        Examples:
            1170 -> "1.1.7.0"
            1903 -> "1.9.0.3"
        """
        major = value // 1000
        minor = (value % 1000) // 100
        patch = (value % 100) // 10
        revision = value % 10
        return f"{major}.{minor}.{patch}.{revision}"

    @property
    def firmware_version_str(self) -> str:
        """Get firmware version as dotted string (e.g., '1.1.7.0')."""
        return self.parse_version(self.firmware_version)

    @property
    def system_version_str(self) -> str:
        """Get system version as dotted string (e.g., '1.9.0.3')."""
        return self.parse_version(self.system_version)

    @property
    def full_version_str(self) -> str:
        """Get combined version string (system.firmware)."""
        return f"{self.system_version_str}.{self.firmware_version_str}"


class MediaFile(SDKModel):
    """Media file metadata from drone storage.

    Represents a photo, video, or log file stored on the drone.
    Retrieved via HTTP CGI API at port 12346.
    """
    name: str = Field(description="Filename (e.g., 20251111_102056_1.jpg)")
    path: str = Field(description="Full path on drone storage")
    thumb_path: Optional[str] = Field(default=None, description="Thumbnail path (photos/videos only)")
    date: str = Field(description="Creation date (e.g., 2025-11-11 10:20:56)")
    size: str = Field(description="File size (e.g., 0.10MB)")
    media_type: MediaType = Field(default=MediaType.PHOTO, description="Type of media file")

    @classmethod
    def from_snap_info(cls, data: dict, media_type: MediaType = MediaType.PHOTO) -> "MediaFile":
        """Create MediaFile from drone's snaplist JSON response."""
        return cls(
            name=data.get("name", ""),
            path=data.get("path", ""),
            thumb_path=data.get("thumb_path"),
            date=data.get("date", ""),
            size=data.get("size", ""),
            media_type=media_type,
        )

    def get_download_url(self, drone_ip: str, port: int = 12346) -> str:
        """Get HTTP URL for downloading this file."""
        # Strip /sdcard prefix from path
        url_path = self.path.replace("/sdcard", "")
        return f"http://{drone_ip}:{port}{url_path}"

    def get_thumb_url(self, drone_ip: str, port: int = 12346) -> Optional[str]:
        """Get HTTP URL for downloading thumbnail."""
        if not self.thumb_path:
            return None
        url_path = self.thumb_path.replace("/sdcard", "")
        return f"http://{drone_ip}:{port}{url_path}"
