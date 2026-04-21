from . import mavlink
from .cmd_params import FormationCmd, Defaults, ModeFlag
from ..core.types import FlipDirection

from datetime import datetime
from ..system import system
from ..system import datacenter
from ..system.command import SysCommand
from . import config

_mavlink = mavlink.MAVLink(
    None, src_system=system.mavlink_system_id, src_component=system.mavlink_component_id
)
_mavlink_file = mavlink.MAVLink(
    None,
    src_system=system.mavlink_system_id,
    src_component=system.mavlink_component_file_id,
)

_data_center = datacenter.DataCenter()


def refresh_runtime_config() -> None:
    """Refresh cached MAVLink encoders after runtime config changes."""
    global _mavlink
    global _mavlink_file

    _mavlink = mavlink.MAVLink(
        None,
        src_system=system.mavlink_system_id,
        src_component=system.mavlink_component_id,
    )
    _mavlink_file = mavlink.MAVLink(
        None,
        src_system=system.mavlink_system_id,
        src_component=system.mavlink_component_file_id,
    )


class CommandProcessor:
    cmd = None

    def __init__(self, cmd):
        self._cmd = cmd

        if _mavlink.srcComponent == 2:
            _mavlink.srcComponent = config.bind_client

        if not self._cmd._data.get("plane_id") is None:
            self._cmd._data["plane_id"] = config.drone_id
        # _mavlink.srcComponent= config.bind_client

    def get_buf(self):
        pass


##LED
def set_rgb(data):
    if data == 0:
        return 0
    # Handle pre-converted int (from new API's to_param4())
    if isinstance(data, int):
        return data
    # Handle dict format (legacy API)
    # Bit layout: [mode:8][b:8][g:8][r:8]
    param4 = data["r"] | (data["g"] << 8) | (data["b"] << 16) | (data["mode"] << 24)
    return param4


##8.Linux
class Plane_Linux_cmd(CommandProcessor):
    cmd = SysCommand.S_Fly_Linux_cmd

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.plane_command_encode(
            data["utc"],
            data["token"],
            data["data"],
            data["plane_id"],
            data["cmd"],
            data["ack"],
            data["type"],
            data["reserve"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFLamplight(CommandProcessor):
    """Set LED color/mode - Formation CMD 26 (SET_RGB).

    Parameters (from mavlink.py):
        param1: duration in 10ms units (100 = 1 second)
        param3: reserved/mode flag (original: 1)
        param4: color packed as `[mode:8][b:8][g:8][r:8]`
    """
    cmd = SysCommand.S_Fly_Lamplight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: duration in 10ms units
        duration = data.get("time", 100)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (original code used 1)
        mode_flag = data.get("mode_flag", 1)
        # param4: LED color encoded [mode:8][b:8][g:8][r:8]
        # Note: different byte order than standard set_rgb()
        r = data.get("r", 0)
        g = data.get("g", 0)
        b = data.get("b", 0)
        led_mode = data.get("mode", 0)
        led_color = (r << 8) | (g << 16) | b | (led_mode << 24)
        # x, y, z, yaw: unused for LED
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            duration,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.SET_RGB,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )

        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTakeoffCP(CommandProcessor):
    """Takeoff - Formation CMD 23 (ONE_KEY_TAKEOFF).

    Parameters (from mavlink.py):
        param1: target height (cm)
        param2: flags (TakeoffFlags bitmask)

            - bit 0: RESET_YAW - reset yaw orientation on takeoff
            - bit 1: WITH_LOAD - takeoff with load/clamp mode
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Takeoff

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: takeoff height in cm
        height = data.get("height", Defaults.TAKEOFF_HEIGHT_CM)
        # param2: takeoff flags (TakeoffFlags bitmask)
        flags = data.get("flags", 0)
        # param3: mode flag (unused for takeoff)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for takeoff
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            height,
            flags,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.ONE_KEY_TAKEOFF,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTouchdownCP(CommandProcessor):
    """Land/Touchdown - Formation CMD 2 (LAND).

    Parameters (from mavlink.py):
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
        All other params unused for basic land command.
    """
    cmd = SysCommand.S_Fly_Touchdown

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: unused for land
        param1 = data.get("param1", 0)
        # param2: unused for land
        param2 = data.get("param2", 0)
        # param3: unused for land
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for land
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.LAND,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFForwardCP(CommandProcessor):
    """Forward movement - Formation CMD 30 (FORWARD_BACK).

    Parameters (from mavlink.py):
        param1: velocity (cm/s)
        x: distance (cm), positive = forward
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Forward

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (100=1m/s, 200=2m/s, 300=3m/s)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward distance in cm (positive = forward)
        distance_x = data.get("distance", 0)
        # y: lateral distance (unused for forward)
        distance_y = data.get("distance_y", 0)
        # z: vertical distance (unused for forward)
        distance_z = data.get("distance_z", 0)
        # yaw: rotation angle (unused for forward)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FORWARD_BACK,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFBackCP(CommandProcessor):
    """Backward movement - Formation CMD 30 (FORWARD_BACK).

    Parameters (from mavlink.py):
        param1: velocity (cm/s)
        x: distance (cm), negative = backward (handled by caller)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Back

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (100=1m/s, 200=2m/s, 300=3m/s)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: backward distance in cm (caller provides positive, library negates)
        distance_x = data.get("distance", 0)
        # y: lateral distance (unused for back)
        distance_y = data.get("distance_y", 0)
        # z: vertical distance (unused for back)
        distance_z = data.get("distance_z", 0)
        # yaw: rotation angle (unused for back)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FORWARD_BACK,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFLeftCP(CommandProcessor):
    """Left movement - Formation CMD 31 (LEFT_RIGHT).

    Parameters (from mavlink.py):
        param1: velocity (cm/s)
        y: distance (cm), positive = left
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Left

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (100=1m/s, 200=2m/s, 300=3m/s)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward distance (unused for left)
        distance_x = data.get("distance_x", 0)
        # y: lateral distance in cm (positive = left)
        distance_y = data.get("distance", 0)
        # z: vertical distance (unused for left)
        distance_z = data.get("distance_z", 0)
        # yaw: rotation angle (unused for left)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.LEFT_RIGHT,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFRightCP(CommandProcessor):
    """Right movement - Formation CMD 31 (LEFT_RIGHT).

    Parameters (from mavlink.py):
        param1: velocity (cm/s)
        y: distance (cm), negative = right (handled by caller)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Right

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (100=1m/s, 200=2m/s, 300=3m/s)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward distance (unused for right)
        distance_x = data.get("distance_x", 0)
        # y: lateral distance in cm (negative = right, handled by caller)
        distance_y = data.get("distance", 0)
        # z: vertical distance (unused for right)
        distance_z = data.get("distance_z", 0)
        # yaw: rotation angle (unused for right)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.LEFT_RIGHT,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFUpCP(CommandProcessor):
    """Upward movement - Formation CMD 29 (UP_DOWN).

    Parameters (from mavlink.py):
        param1: velocity (cm/s) - NOTE: firmware uses fixed MPC_PM_Z_V_UP
        z: distance/height (cm), positive = up
        param4: LED color (r | g<<8 | b<<16 | mode<<24)

    Note: Vertical velocity is controlled by firmware params, not API speed.
    Firmware limits: ascent 1.5 m/s (MPC_PM_Z_V_UP), descent 1.0 m/s (MPC_PM_Z_V_DOWN)
    """
    cmd = SysCommand.S_Fly_Up

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (firmware may override for vertical)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward distance (unused for up)
        distance_x = data.get("distance_x", 0)
        # y: lateral distance (unused for up)
        distance_y = data.get("distance_y", 0)
        # z: vertical distance in cm (positive = up)
        distance_z = data.get("height", 0)
        # yaw: rotation angle (unused for up)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.UP_DOWN,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFDownCP(CommandProcessor):
    """Downward movement - Formation CMD 29 (UP_DOWN).

    Parameters (from mavlink.py):
        param1: velocity (cm/s) - NOTE: firmware uses fixed MPC_PM_Z_V_DOWN
        z: distance/height (cm), negative = down (handled by caller)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)

    Note: Vertical velocity is controlled by firmware params, not API speed.
    Firmware limits: descent 1.0 m/s (MPC_PM_Z_V_DOWN)
    """
    cmd = SysCommand.S_Fly_Down

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (firmware may override for vertical)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for this command
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal, 2=one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward distance (unused for down)
        distance_x = data.get("distance_x", 0)
        # y: lateral distance (unused for down)
        distance_y = data.get("distance_y", 0)
        # z: vertical distance in cm (negative = down, handled by caller)
        distance_z = data.get("height", 0)
        # yaw: rotation angle (unused for down)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.UP_DOWN,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTurnLeftCP(CommandProcessor):
    """Turn left (yaw CCW) - Formation CMD 32 (YAW_ANGLE).

    Parameters (from mavlink.py):
        yaw: angle in degrees (positive = CCW)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_TurnLeft

    def get_buf(self):
        data = self._cmd.get_data()

        # param1-3: unused for yaw
        param1 = data.get("param1", 0)
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z: unused for yaw
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        # yaw: rotation angle in degrees (positive = CCW/left)
        yaw_angle = data.get("angle", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.YAW_ANGLE,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTurnRightCP(CommandProcessor):
    """Turn right (yaw CW) - Formation CMD 32 (YAW_ANGLE).

    Parameters (from mavlink.py):
        yaw: angle in degrees (negative = CW)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_TurnRight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1-3: unused for yaw
        param1 = data.get("param1", 0)
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z: unused for yaw
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        # yaw: rotation angle in degrees (negative = CW/right, caller handles sign)
        yaw_angle = data.get("angle", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.YAW_ANGLE,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTurnLeft360CP(CommandProcessor):
    """360-degree turn left (CCW) - Formation CMD 36 (YAW_TURN).

    Parameters (from mavlink.py):
        param1: number of 360-degree turns
        param3: mode flag (2 = one_key_function required)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_TurnLeft360

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: number of full rotations
        num_turns = data.get("num", 1)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (must be 2 for one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.ONE_KEY_FUNCTION)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z: unused for yaw turn
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        # yaw: unused (direction is implied by num sign)
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            num_turns,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.YAW_TURN,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFTurnRight360CP(CommandProcessor):
    """360-degree turn right (CW) - Formation CMD 36 (YAW_TURN).

    Parameters (from mavlink.py):
        param1: number of 360-degree turns (negative for CW)
        param3: mode flag (0 = standard for CW direction)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_TurnRight360

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: number of full rotations
        num_turns = data.get("num", 1)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (0 for CW direction)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z: unused for yaw turn
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        # yaw: unused
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            num_turns,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.YAW_TURN,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFBounceCP(CommandProcessor):
    """Bounce maneuver - Formation CMD 33 (BOUNCE).

    Parameters (from mavlink.py):
        param1: count/frequency of bounces
        param3: mode flag (2 = one_key_function required)
        z: bounce height (cm)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_Bounce

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: number of bounces
        count = data.get("frequency", 1)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (must be 2 for one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.ONE_KEY_FUNCTION)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y: unused for bounce
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        # z: bounce height in cm
        bounce_height = data.get("height", 50)
        # yaw: unused
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            count,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            bounce_height,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.BOUNCE,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFStraightFlightCP(CommandProcessor):
    """3D position flight - Formation CMD 35 (STRAIGHT_FLIGHT).

    Parameters (from mavlink.py):
        param1: velocity (cm/s)
        x: forward/backward distance (cm)
        y: left/right distance (cm)
        z: up/down distance (cm)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_StraightFlight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity in cm/s (100=1m/s, 200=2m/s, 300=3m/s)
        velocity = data.get("speed", Defaults.SPEED_CMS)
        # param2: unused for straight flight
        param2 = data.get("param2", 0)
        # param3: mode flag (0=normal)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x: forward/backward distance in cm (must be int for MAVLink struct)
        distance_x = int(data.get("x", 0))
        # y: left/right distance in cm (must be int for MAVLink struct)
        distance_y = int(data.get("y", 0))
        # z: up/down distance in cm (must be int for MAVLink struct)
        distance_z = int(data.get("z", 0))
        # yaw: rotation angle (must be int for MAVLink struct)
        yaw_angle = int(data.get("yaw", 0))

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.STRAIGHT_FLIGHT,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFFlipForwardCP(CommandProcessor):
    """Forward flip - Formation CMD 37 (FLIP).

    Parameters (from mavlink.py):
        param1: direction (0=forward, 1=back, 2=left, 3=right)
        param3: mode flag (2 = one_key_function required)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_FlipForward

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: flip direction (from caller, default forward)
        direction = data.get("direction", FlipDirection.FORWARD)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (must be 2 for one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.ONE_KEY_FUNCTION)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for flip
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            direction,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FLIP,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFFlipBackCP(CommandProcessor):
    """Backward flip - Formation CMD 37 (FLIP).

    Parameters (from mavlink.py):
        param1: direction (1 = back)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_FlipBack

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: flip direction (back = 1)
        direction = data.get("direction", FlipDirection.BACK)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (note: original code had 0 here)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for flip
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            direction,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FLIP,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFFlipLeftCP(CommandProcessor):
    """Left flip - Formation CMD 37 (FLIP).

    Parameters (from mavlink.py):
        param1: direction (2 = left)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_FlipLeft

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: flip direction (left = 2)
        direction = data.get("direction", FlipDirection.LEFT)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (note: original code had 0 here)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for flip
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            direction,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FLIP,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFFlipRightCP(CommandProcessor):
    """Right flip - Formation CMD 37 (FLIP).

    Parameters (from mavlink.py):
        param1: direction (3 = right)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_FlipRight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: flip direction (right = 3)
        direction = data.get("direction", FlipDirection.RIGHT)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (note: original code had 0 here)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for flip
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            direction,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.FLIP,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFCurvilinearFlight(CommandProcessor):
    """Curved/arc flight - Formation CMD 38 (CURVE_FLIGHT).

    Parameters (from mavlink.py):
        param1: velocity (positive=CW, negative=CCW arc)
        param3: mode flag (2 = one_key_function required)
        x, y, z: endpoint coordinates (cm)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_CurvilinearFlight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: velocity with direction sign (100=CW, -100=CCW)
        if data.get("direction", True):
            velocity = data.get("velocity", 100)
        else:
            velocity = data.get("velocity", -100)
        # param2: curve mode (unused in original)
        param2 = data.get("param2", 0)
        # param3: mode flag (must be 2 for one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.ONE_KEY_FUNCTION)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z: endpoint coordinates in cm
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        # yaw: unused for curve flight
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            velocity,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.CURVE_FLIGHT,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFHoverFlight(CommandProcessor):
    """Hover/hold position - Formation CMD 46 (HOLD).

    Parameters (from mavlink.py):
        param1: duration in seconds (0 = indefinite)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_HoverFlight

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: hold duration in seconds
        duration = data.get("time", Defaults.HOLD_DURATION_SEC)
        # param2-3: unused
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for hold
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            duration,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.HOLD,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFBarrier_aircraft(CommandProcessor):
    """Enable/disable obstacle avoidance - Formation CMD 40/41.

    True enables avoidance (CMD 40). False disables it (CMD 41).
    """
    cmd = SysCommand.S_Fly_Barrier_aircraft

    def get_buf(self):
        data = self._cmd.get_data()

        # Determine command based on mode
        enable_mode = data.get("mode", True)
        cmd_id = FormationCmd.ENABLE_AVOIDANCE if enable_mode else FormationCmd.DISABLE_AVOIDANCE

        # param1-4: unused for basic enable/disable
        param1 = data.get("param1", 0)
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        led_color = data.get("led_color", 0)
        # x, y, z, yaw: unused
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            cmd_id,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFLine_walking(CommandProcessor):
    cmd = SysCommand.S_Fly_Line_walking

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.line_walking_encode(
            data["fun_id"], data["dist"], data["tv"], data["way_color"], 0
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFAiIdentifies(CommandProcessor):
    cmd = SysCommand.S_Fly_AiIdentifies

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.camera_encode(data["mode"], 0, 0, 0, 0, 0, 0, 0)
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFQr_code_tracking(CommandProcessor):
    cmd = SysCommand.S_Fly_Qr_tracking

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.camera_encode(
            data["mode"], data["type"], 0, 0, 0, 0, 0, data["time_duration"]
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFQr_code_aligns(CommandProcessor):
    cmd = SysCommand.S_Fly_Qr_align

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.qrrecognite_deal_encode(
            data["time_duration"],
            data["search_radius"],
            20,
            0,
            data["qr_id"],
            0,
            0,
            0,
            0,
            data["mode"],
            0,
            0,
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_ColorRecog(CommandProcessor):
    cmd = SysCommand.S_Fly_ColorRecog

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.colorrecog_encode(data["Mode"], 0, 0, 0, 0)
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_unlock(CommandProcessor):
    """Arm motors - Formation CMD 4 (ARM).

    Unlocks/arms the drone motors for flight.
    """
    cmd = SysCommand.S_Fly_unlock

    def get_buf(self):
        data = self._cmd.get_data()

        # All params unused for arm command
        param1 = data.get("param1", 0)
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        led_color = data.get("led_color", 0)
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.ARM,
            data.get("option", 1),  # Original code had option=1
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_lock(CommandProcessor):
    """Disarm motors - Formation CMD 5 (DISARM).

    Locks/disarms the drone motors.
    """
    cmd = SysCommand.S_Fly_lock

    def get_buf(self):
        data = self._cmd.get_data()

        # All params unused for disarm command
        param1 = data.get("param1", 0)
        param2 = data.get("param2", 0)
        mode_flag = data.get("mode_flag", ModeFlag.NORMAL)
        led_color = data.get("led_color", 0)
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            param1,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.DISARM,
            data.get("option", 1),  # Original code had option=1
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SFCircumvolant(CommandProcessor):
    """Circle/orbit maneuver - Formation CMD 34 (CIRCLE).

    Parameters (from mavlink.py):
        param1: radius in cm
        param3: mode flag (2 = one_key_function required)
        param4: LED color (r | g<<8 | b<<16 | mode<<24)
    """
    cmd = SysCommand.S_Fly_RadiusAround

    def get_buf(self):
        data = self._cmd.get_data()

        # param1: circle radius in cm
        radius = data.get("radius", 50)
        # param2: unused
        param2 = data.get("param2", 0)
        # param3: mode flag (must be 2 for one_key_function)
        mode_flag = data.get("mode_flag", ModeFlag.ONE_KEY_FUNCTION)
        # param4: LED color encoded
        led_color = set_rgb(data.get("led", 0))
        # x, y, z, yaw: unused for circle
        distance_x = int(data.get("x", 0))
        distance_y = int(data.get("y", 0))
        distance_z = int(data.get("z", 0))
        yaw_angle = data.get("yaw", 0)

        msg = _mavlink.formation_cmd_encode(
            radius,
            param2,
            mode_flag,
            led_color,
            distance_x,
            distance_y,
            distance_z,
            yaw_angle,
            data["plane_id"],
            data["plane_id"],
            FormationCmd.CIRCLE,
            data.get("option", Defaults.OPTION),
            data.get("index", Defaults.INDEX),
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Plane_time(CommandProcessor):
    cmd = SysCommand.S_Fly_Plane_time

    #
    def get_buf(self):
        data = self._cmd.get_data()
        now = datetime.now()

        #
        formatted_time = now.strftime("%Y.%m.%d %H:%M:%S")

        print(f"[DEBUG] SF_Plane_time data: {data}")
        print(f"[DEBUG] token type: {type(data.get('token'))}, value: {data.get('token')}")
        print(f"[DEBUG] plane_id type: {type(data.get('plane_id'))}, value: {data.get('plane_id')}")

        msg = _mavlink.plane_ack_extend_encode(
            data["token"], data["plane_id"], 25, 0, 0, formatted_time.encode()
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Enable_LED(CommandProcessor):
    """Enable LED - formation cmd 0x0C (12)"""
    cmd = SysCommand.S_Fly_Enable_LED

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            12,  # cmd = ENABLE_LED (0x0C)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Disable_LED(CommandProcessor):
    """Disable LED - formation cmd 0x0D (13)"""
    cmd = SysCommand.S_Fly_Disable_LED

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            13,  # cmd = DISABLE_LED (0x0D)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Cancel_RGB(CommandProcessor):
    """Cancel RGB animation - formation cmd 0x1B (27)"""
    cmd = SysCommand.S_Fly_Cancel_RGB

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            27,  # cmd = CANCEL_RGB (0x1B)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Vertical_Circle(CommandProcessor):
    """Vertical circle maneuver - formation cmd 0x27 (39)

    Firmware: set_vertical_circle_mode_param(radius * 0.01)
    Requires altitude >= 0.35m
    """
    cmd = SysCommand.S_Fly_Vertical_Circle

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            data["radius"],  # param1: radius in cm
            0,  # param2
            2,  # param3: one_key_function flag
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            39,  # cmd = VERTICAL_CIRCLE (0x27)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Avoidance(CommandProcessor):
    """Set avoidance with direction - formation cmd 0x2A (42)

    Firmware: Moves in specified direction when obstacle detected
    Direction: 0=forward, 1=back, 2=left, 3=right, 4/5=up/down
    """
    cmd = SysCommand.S_Fly_Set_Avoidance

    def get_buf(self):
        data = self._cmd.get_data()
        # param1 layout: [direction_byte << 8] | barrier_mask
        # direction in high byte (bits 16-23 when shifted)
        direction = data.get("direction", 0)
        barrier_mask = data.get("barrier_mask", 0x3F)  # all directions by default
        param1 = (direction << 8) | barrier_mask
        msg = _mavlink.formation_cmd_encode(
            param1,  # param1: direction and barrier mask
            0,  # param2
            0,  # param3
            0,  # param4
            int(data.get("x", 0)),  # x: forward/back distance
            int(data.get("y", 0)),  # y: left/right distance
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            42,  # cmd = SET_AVOIDANCE (0x2A)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Get_Product_ID(CommandProcessor):
    """Get product ID/autopilot version - formation cmd 0x2C (44)

    Firmware: send_autopilot_version()
    """
    cmd = SysCommand.S_Fly_Get_Product_ID

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            44,  # cmd = GET_PRODUCT_ID (0x2C)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Velocity(CommandProcessor):
    """Set velocity level - formation cmd 0x30 (48)

    Firmware: set_manual_velocity(level), set_manual_horizontal_vel(horiz_vel)
    Level is velocity in cm/s (0-300): 100=1.0m/s, 200=2.0m/s, 300=3.0m/s
    """
    cmd = SysCommand.S_Fly_Set_Velocity

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            data["level"],  # param1: velocity in cm/s (0-300)
            data.get("horizontal_vel", 0),  # param2: horizontal velocity override
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            48,  # cmd = SET_VELOCITY (0x30)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Yawrate(CommandProcessor):
    """Set yaw rate level - formation cmd 0x31 (49)

    Firmware: set_manual_yaw_rate(level)
    """
    cmd = SysCommand.S_Fly_Set_Yawrate

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            data["level"],  # param1: yaw rate level
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            49,  # cmd = SET_YAWRATE (0x31)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_RGB_Brightness(CommandProcessor):
    """Set RGB brightness - formation cmd 0x32 (50)

    Firmware: set_rgb_brightness(brightness)
    """
    cmd = SysCommand.S_Fly_Set_RGB_Brightness

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            data["brightness"],  # param1: brightness level
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            50,  # cmd = SET_RGB_BRIGHTNESS (0x32)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Enable_Battery_FS(CommandProcessor):
    """Enable battery failsafe - formation cmd 0x35 (53)

    Firmware: set_battery_failsafe(1)
    """
    cmd = SysCommand.S_Fly_Enable_Battery_FS

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            53,  # cmd = ENABLE_BATTERY_FS (0x35)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Disable_Battery_FS(CommandProcessor):
    """Disable battery failsafe - formation cmd 0x36 (54)

    Firmware: set_battery_failsafe(0)
    """
    cmd = SysCommand.S_Fly_Disable_Battery_FS

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            54,  # cmd = DISABLE_BATTERY_FS (0x36)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Parameter(CommandProcessor):
    """Set multiple parameters - formation cmd 0x37 (55)

    Firmware packing:

    - param1 byte 0: velocity level
    - param1 byte 1: yaw rate level
    - param1 byte 2: brightness
    - param2 byte 0: avoidance (0=off, 1=on)
    - param2 byte 1: battery failsafe (0=off, 1=on)
    - param2 byte 2: fast land (0=slow, 1=fast)
    """
    cmd = SysCommand.S_Fly_Set_Parameter

    def get_buf(self):
        data = self._cmd.get_data()
        # Pack param1: [brightness:8][yaw_rate:8][velocity:8]
        velocity = data.get("velocity", 0) & 0xFF
        yaw_rate = data.get("yaw_rate", 0) & 0xFF
        brightness = data.get("brightness", 0) & 0xFF
        param1 = velocity | (yaw_rate << 8) | (brightness << 16)

        # Pack param2: [fast_land:8][batt_fs:8][avoidance:8]
        avoidance = 1 if data.get("avoidance", False) else 0
        batt_fs = 1 if data.get("battery_failsafe", False) else 0
        fast_land = 1 if data.get("fast_land", False) else 0
        param2 = avoidance | (batt_fs << 8) | (fast_land << 16)

        msg = _mavlink.formation_cmd_encode(
            param1,  # param1: velocity, yaw_rate, brightness
            param2,  # param2: avoidance, batt_fs, fast_land
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            55,  # cmd = SET_PARAMETER (0x37)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Operate(CommandProcessor):
    """Set formation operate status - formation cmd 0x3B (59)

    Firmware: set_formation_operate_status(status)
    """
    cmd = SysCommand.S_Fly_Operate

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            data["status"],  # param1: operate status
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            59,  # cmd = OPERATE (0x3B)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Land_Speed(CommandProcessor):
    """Set land speed - formation cmd 0x3C (60)

    Firmware: set_fast_land(fast ? 1 : 0)
    0 = slow landing, 1 = fast landing
    """
    cmd = SysCommand.S_Fly_Set_Land_Speed

    def get_buf(self):
        data = self._cmd.get_data()
        msg = _mavlink.formation_cmd_encode(
            1 if data.get("fast", False) else 0,  # param1: 0=slow, 1=fast
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # x
            0,  # y
            0,  # z
            0,  # yaw
            data["plane_id"],
            data["plane_id"],
            60,  # cmd = SET_LAND_SPEED (0x3C)
            0,  # option
            0,  # index
            data["token"],
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_Video_Resolution(CommandProcessor):
    """Set video resolution - plane cmd 0x16 (22)

    Firmware handler: HandleMsgSelectRecordResolution @ avmanager:0x0001df7c
    Controls RTP streaming and recording resolution.

    Resolution levels:

    - 0 (HIGH): 1920x1080 (1080p)
    - 1 (MEDIUM): 1280x720 (720p)
    - 2 (LOW): 640x480 or lower (program/AI mode)

    Lower resolution = less encoder CPU = potentially better QR rate during RTP.
    """
    cmd = SysCommand.S_Fly_Set_Video_Resolution

    def get_buf(self):
        data = self._cmd.get_data()
        now = datetime.now()
        utc = int(now.timestamp())

        msg = _mavlink.plane_command_encode(
            utc,                        # utc timestamp
            data["token"],              # token
            data["resolution"],         # data: resolution level (0, 1, 2)
            data["plane_id"],           # plane_id
            22,                         # cmd: MAV_PLANE_CMD_SELECT_VIDEO_RESOLUTION (0x16)
            1,                          # ack: expect acknowledgment
            0,                          # message_type
            0,                          # reserve
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


class SF_Set_WiFi_Mode(CommandProcessor):
    """Set WiFi mode - plane cmd 0x04 (4)

    Firmware handler: HandleMsgWifiMode @ avmanager

    WiFi mode values (type parameter):

    - 0: Switch to 2.4GHz band
    - 1: Switch to 5GHz band
    - 2: Switch to AP mode
    - 3: Low WiFi power
    - 4: High WiFi power
    - 5: WiFi broadcast ON
    - 6: WiFi broadcast OFF
    - 7: Manual channel mode (channel_id in data)
    - 8: Auto channel mode
    - 9: Get channel strength
    """
    cmd = SysCommand.S_Fly_Set_WiFi_Mode

    def get_buf(self):
        data = self._cmd.get_data()
        now = datetime.now()
        utc = int(now.timestamp())

        msg = _mavlink.plane_command_encode(
            utc,                        # utc timestamp
            data["token"],              # token
            data.get("channel_id", 0),  # data: channel_id for CHANNEL_MANUAL mode
            data["plane_id"],           # plane_id
            4,                          # cmd: MAV_PLANE_CMD_WIFI_MODE (0x04)
            1,                          # ack: expect acknowledgment
            data["wifi_mode"],          # message_type: WiFi mode (0-9)
            0,                          # reserve
        )
        _mavlink.send(msg)
        buf = msg.pack(_mavlink)
        return buf


command_processor_list = [
    SFTakeoffCP,
    SFTouchdownCP,
    SFForwardCP,
    SFBackCP,
    SFLeftCP,
    SFRightCP,
    SFUpCP,
    SFDownCP,
    SFTurnLeftCP,
    SFTurnRightCP,
    SFTurnLeft360CP,
    SFTurnRight360CP,
    SFBounceCP,
    SFStraightFlightCP,
    SFFlipForwardCP,
    SFFlipBackCP,
    SFFlipLeftCP,
    SFFlipRightCP,
    SFLamplight,
    Plane_Linux_cmd,
    SFCurvilinearFlight,
    SFHoverFlight,
    SFBarrier_aircraft,
    SFLine_walking,
    SFAiIdentifies,
    SFQr_code_tracking,
    SFQr_code_aligns,
    SF_ColorRecog,
    SF_unlock,
    SF_lock,
    SFCircumvolant,
    SF_Plane_time,
    SF_Enable_LED,
    SF_Disable_LED,
    SF_Cancel_RGB,
    SF_Vertical_Circle,
    SF_Set_Avoidance,
    SF_Get_Product_ID,
    SF_Set_Velocity,
    SF_Set_Yawrate,
    SF_Set_RGB_Brightness,
    SF_Enable_Battery_FS,
    SF_Disable_Battery_FS,
    SF_Set_Parameter,
    SF_Operate,
    SF_Set_Land_Speed,
    SF_Set_Video_Resolution,
    SF_Set_WiFi_Mode,
]


class CommandProcessorFactory:
    def get_command_processor(ucmd):
        for cl in command_processor_list:
            if cl.cmd == ucmd.get_command():
                return cl(ucmd)
        return None
