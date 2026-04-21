"""
Type definitions and enums for Drone API.

These enums replace magic numbers throughout the codebase with type-safe constants.
"""

from enum import IntEnum, IntFlag


class Direction(IntEnum):
    """Movement direction for move() command.

    FORWARD/BACK map to MAVLink command ID 30 with +/- distance.

    LEFT/RIGHT map to MAVLink command ID 31 with +/- distance.

    UP/DOWN map to MAVLink command ID 32 with +/- height.
    """
    FORWARD = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    UP = 4
    DOWN = 5


class Rotation(IntEnum):
    """Rotation direction for rotate() command."""
    LEFT = 0   # Counter-clockwise (positive angle)
    RIGHT = 1  # Clockwise (negative angle)


class FlipDirection(IntEnum):
    """Direction for flip/somersault maneuver.

    Maps to the direction parameter in single_fly_somersault().
    """
    FORWARD = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3


class LEDMode(IntEnum):
    """LED lighting mode.

    Maps to the mode parameter in single_fly_lamplight().
    """
    CONSTANT = 1
    OFF = 2
    RGB_CYCLE = 4
    SEVEN_COLOR = 16
    BLINK = 32
    BREATHING = 64


class DroneStatus(IntEnum):
    """Drone operational status from telemetry."""
    DISCONNECTED = 0
    CONNECTED = 1
    READY = 2
    FLYING = 3
    LOW_BATTERY = 4
    ERROR = 5


class CommandResult(IntEnum):
    """Result code from command execution.

    Based on MAVLink PLANE_ACK message result codes.
    """
    SUCCESS = 255
    BUSY = 240
    FAILED_241 = 241
    FAILED_242 = 242
    TIMEOUT = -1
    UNKNOWN = -2


class VisionMode(IntEnum):
    """Camera mode for vision/QR operations.

    Selects between optical flow (downward) and front camera.
    """
    OPTICAL_FLOW = 0
    FRONT_CAMERA = 1


class AIRecognitionTarget(IntEnum):
    """Target types for AI recognition.

    Used with single_fly_AiIdentifies() for recognizing visual markers.
    """
    # Digits
    DIGIT_0 = 0
    DIGIT_1 = 1
    DIGIT_2 = 2
    DIGIT_3 = 3
    DIGIT_4 = 4
    DIGIT_5 = 5
    DIGIT_6 = 6
    DIGIT_7 = 7
    DIGIT_8 = 8
    DIGIT_9 = 9
    # Arrows
    ARROW_LEFT = 10
    ARROW_RIGHT = 11
    ARROW_UP = 12
    ARROW_DOWN = 13
    # Special
    END_TASK = 20
    # Letters A-Z are 65-90 (ASCII codes)


class CameraMode(IntEnum):
    """Camera angle control mode.

    Used with Plane_cmd_camera_angle().
    """
    RESET = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
    SET_ANGLE = 5
    GET_ANGLE = 6


class ClampMode(IntEnum):
    """Gripper/clamp control mode.

    Used with MAV_PLANE_CMD_CLAMP_ELECTROMAGNET (cmd=26).

    Type values:

    - 0=CLOSE
    - 1=OPEN
    - 2=SET_ANGLE (with data=angle)
    """
    CLOSE = 0
    OPEN = 1
    SET_ANGLE = 2  # Use with angle in data field


class ElectromagnetMode(IntEnum):
    """Electromagnet control mode.

    Used with MAV_PLANE_CMD_CLAMP_ELECTROMAGNET (cmd=26).

    Type values: 3=OFF, 4=ON
    """
    OFF = 3
    ON = 4


class VideoMode(IntEnum):
    """Video recording control.

    Used with Plane_cmd_switch_video().
    """
    START = 0
    STOP = 1


class LineWalkingMode(IntEnum):
    """Line walking/following mode.

    Used with single_fly_Line_walking().
    """
    START = 1
    STOP = 0


class BarrierMode(IntEnum):
    """Obstacle avoidance mode.

    Used with single_fly_barrier_aircraft().
    """
    DISABLE = 0
    ENABLE = 1


class LineFollowResult(IntEnum):
    """Result from line following operation.

    Used with follow_line() / single_fly_Line_walking().
    """
    FAILED = 0
    SUCCESS = 1
    INTERSECTION = 2  # Successfully encountered intersection


class LineColor(IntEnum):
    """Line color for line following.

    Used with follow_line() to specify the line color to track.
    """
    BLACK = 0
    WHITE = 255


class LaserMode(IntEnum):
    """Laser firing mode.

    Used with fire_laser() / plane_fly_generating().
    """
    SINGLE_SHOT = 0
    BURST = 1
    RECEIVER_ON = 2
    RECEIVER_OFF = 3
    CONTINUOUS = 4  # Continuous fire, no ammo consumption
    OFF = 5


class CameraPitchMode(IntEnum):
    """Camera pitch control mode.

    Used with set_camera_angle() / Plane_cmd_camera_angle().

    Controls the main camera's vertical angle.
    """
    UP_ABSOLUTE = 0      # Rotate up to absolute angle (0-90)
    DOWN_ABSOLUTE = 1    # Rotate down to absolute angle (0-90)
    # 2, 3 are reserved for algorithm control
    CALIBRATE = 4        # Calibration mode
    UP_RELATIVE = 5      # Rotate up relative (blocked movement)
    DOWN_RELATIVE = 6    # Rotate down relative (blocked movement)


class VideoStreamMode(IntEnum):
    """Video stream control mode.

    Used with set_video_stream() / Plane_cmd_swith_rtp().
    """
    ENABLE = 0
    DISABLE = 1


class QRLocalizationMode(IntEnum):
    """QR code localization mode.

    Used with set_qr_localization() / Plane_cmd_switch_QR().
    """
    ENABLE = 0
    DISABLE = 1


class MediaType(IntEnum):
    """Media type for file operations.

    Used with list_media(), download_media(), delete_media().

    Maps to the media_type/del_type parameters in the HTTP CGI API.
    """
    PHOTO = 0
    VIDEO = 1
    LOG = 2


class VelocityLevel(IntEnum):
    """Flight velocity level for position controller gain scaling.

    Used with movement commands like move() and move_to() to control flight speed.
    Pass as the `speed` parameter:

    ```python
    drone.move(Direction.FORWARD, 100, speed=VelocityLevel.ZOOM)
    drone.move_to(x=100, y=100, z=50, speed=VelocityLevel.TURBO)
    ```

    Firmware behavior:

    The firmware uses this value as a DIVISOR for the position P-gain:

    `gain = MPC_XY_P (2.0) / cruising_speed`

    Higher values -> lower gain -> slower, gentler approach

    Lower values -> higher gain -> faster, more aggressive approach

    Note:
        set_velocity_level() only affects RC/joystick manual control, not API commands.
    """
    SLOW = 300    # P-gain 0.67 - slow, smooth movement
    MEDIUM = 200  # P-gain 1.0 - balanced
    ZOOM = 100    # P-gain 2.0 - fast (default)
    TURBO = 50    # P-gain 4.0 - very fast, aggressive


class BarrierMask(IntFlag):
    """Obstacle sensor bitmask for set_avoidance_direction().

    Specifies which IR/ToF sensors to check for obstacles.

    Can be combined with `|` operator: `BarrierMask.FRONT | BarrierMask.BACK`

    Maps to mav_barrier_info in firmware.
    """
    FRONT = 0x01  # Bit 0: Front sensor
    BACK = 0x02   # Bit 1: Back sensor
    LEFT = 0x04   # Bit 2: Left sensor
    RIGHT = 0x08  # Bit 3: Right sensor
    UP = 0x10     # Bit 4: Up sensor
    DOWN = 0x20   # Bit 5: Down sensor

    # Common combinations
    HORIZONTAL = FRONT | BACK | LEFT | RIGHT  # 0x0F - All horizontal sensors
    ALL = FRONT | BACK | LEFT | RIGHT | UP | DOWN  # 0x3F - All sensors


class VideoResolution(IntEnum):
    """Video resolution level for RTP streaming and recording.

    Used with set_video_resolution() to control video quality vs performance.

    Lower resolution reduces encoder CPU load, which may allow higher QR
    localization rates when RTP streaming is active.

    Firmware handler: `HandleMsgSelectRecordResolution @ avmanager:0x0001df7c`

    Command: MAV_PLANE_CMD_SELECT_VIDEO_RESOLUTION (0x16)

    Resolution affects:

    - RTP streaming quality and bandwidth
    - Video recording file size
    - Encoder CPU load (lower = less load)
    """
    HIGH = 0      # 1920x1080 (1080p) - Default viewing
    MEDIUM = 1    # 1280x720 (720p) - Battery saving
    LOW = 2       # 640x480 or lower - AI/Programming mode


class WiFiMode(IntEnum):
    """WiFi configuration mode.

    Used with MAV_PLANE_CMD_WIFI_MODE (cmd=4).
    The type parameter selects the WiFi operation.

    Firmware handler: HandleMsgWifiMode @ avmanager
    """
    BAND_2_4GHZ = 0        # Switch to 2.4GHz band
    BAND_5GHZ = 1          # Switch to 5GHz band
    AP_MODE = 2            # Switch to Access Point mode
    POWER_LOW = 3          # Low WiFi transmission power
    POWER_HIGH = 4         # High WiFi transmission power
    BROADCAST_ON = 5       # Enable WiFi broadcast
    BROADCAST_OFF = 6      # Disable WiFi broadcast
    CHANNEL_MANUAL = 7     # Manual channel selection (requires channel_id in data)
    CHANNEL_AUTO = 8       # Automatic channel selection
    GET_CHANNEL_STRENGTH = 9  # Query WiFi channel signal strength


class TakeoffFlags(IntFlag):
    """Takeoff behavior flags (param2 in ONE_KEY_TAKEOFF command).

    Controls special takeoff behaviors. Can be combined with | operator.

    Firmware behavior (handle_formation_cmd_handler @ 0x00043424):

        - Bit 0 (RESET_YAW): Resets yaw orientation on takeoff
        - Bit 1 (WITH_LOAD): Enables "takeoff with load" mode for heavier payloads

    C# reference (SingleGameProgrammingAdapter.cs):

        - LaunchHeight (param2=0): Normal takeoff
        - LaunchHeightWithClamp (param2=2): Takeoff with load clamp

    Usage:
    ```python
    drone.takeoff(height_cm=100, flags=TakeoffFlags.WITH_LOAD)
    drone.takeoff(height_cm=100, flags=TakeoffFlags.RESET_YAW | TakeoffFlags.WITH_LOAD)
    ```
    """
    NONE = 0           # Normal takeoff (default)
    RESET_YAW = 0x01   # Bit 0: Reset yaw orientation on takeoff
    WITH_LOAD = 0x02   # Bit 1: Takeoff with load/clamp - may have different dynamics
