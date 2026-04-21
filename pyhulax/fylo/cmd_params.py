"""
Formation Command Parameter Documentation.

Maps MAVLink formation_cmd_encode parameters to named constants.
Based on mavlink.py enum documentation and firmware analysis.

formation_cmd_encode signature:
    formation_cmd_encode(param1, param2, param3, param4, x, y, z, yaw,
                         start_id, end_id, cmd, ack, command_type, token)

Parameter semantics vary by command - see mavlink.py enums['MAV_FORMATION_CMD'][ID].param[]
"""


class FormationCmd:
    """Formation command IDs from MAV_FORMATION_CMD enum."""

    LAND = 2
    ARM = 4
    DISARM = 5
    ENABLE_LED = 12
    DISABLE_LED = 13
    ONE_KEY_TAKEOFF = 23
    SET_RGB = 26
    CANCEL_RGB = 27
    MISSION = 28
    UP_DOWN = 29
    FORWARD_BACK = 30
    LEFT_RIGHT = 31
    YAW_ANGLE = 32
    BOUNCE = 33
    CIRCLE = 34
    STRAIGHT_FLIGHT = 35
    YAW_TURN = 36
    FLIP = 37
    CURVE_FLIGHT = 38
    VERTICAL_CIRCLE = 39
    ENABLE_AVOIDANCE = 40
    DISABLE_AVOIDANCE = 41
    SET_AVOIDANCE = 42
    GET_PRODUCT_ID = 44
    HOLD = 46
    SET_VELOCITY = 48
    SET_YAWRATE = 49
    SET_RGB_BRIGHTNESS = 50
    ENABLE_BATTERY_FS = 53
    DISABLE_BATTERY_FS = 54
    SET_PARAMETER = 55
    OPERATE = 59
    SET_LAND_SPEED = 60
    # AI/Vision commands
    AI_IDENTIFIES = 63
    LINE_WALKING = 64
    QR_CODE_LOCATION = 65
    QR_CODE_SCANNING = 66
    COLOR_RECOGNITION = 67
    BARRIER_AIRCRAFT = 68


class Defaults:
    """Default values for command parameters."""

    SPEED_CMS = 100  # 1.0 m/s in cm/s
    OPTION = 0  # No special options
    INDEX = 0  # Command index
    ACK = 0  # No ack request
    COMMAND_TYPE = 0  # Broadcast type
    ONE_KEY_FLAG = 2  # param3 flag for one-key maneuvers (bounce, flip, etc.)
    TAKEOFF_HEIGHT_CM = 80  # Default takeoff height
    HOLD_DURATION_SEC = 0  # Indefinite hold


class ModeFlag:
    """param3 mode flags for various commands."""

    NORMAL = 0
    ONE_KEY_FUNCTION = 2  # Required for bounce, flip, circle, yaw_turn, etc.


class CurveMode:
    """param2 curve mode for CURVE_FLIGHT (cmd 38)."""

    CLOCKWISE = 0
    COUNTER_CLOCKWISE = 1


# FlipDirection is defined in pyhulax.core.types - import from there
