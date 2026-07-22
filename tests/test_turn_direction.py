"""Regression tests for yaw turn direction (turn-right must yaw CW, not CCW).

FORMATION_CMD encodes yaw as a signed int16 where positive = CCW/left on this
airframe. turn-left must emit a positive yaw and turn-right a negative one; when
they were identical, single_fly_turnright(15) yawed left and retrace's turn-left
inverse repeated the turn instead of undoing it.
"""

from pyhulax.fylo import mavlink
from pyhulax.fylo.commandprocessor import CommandProcessorFactory
from pyhulax.system.command import Command, SysCommand


def _turn(cmd, angle):
    return Command(cmd, {"plane_id": 1, "token": 5, "angle": angle, "led": 0})


def _encoder():
    return mavlink.MAVLink(None, src_system=255, src_component=94)


def _yaw(cmd, angle):
    cp = CommandProcessorFactory.get_command_processor(
        _turn(cmd, angle), _encoder(), drone_id=1
    )
    return mavlink.MAVLink(None).decode(bytearray(cp.get_buf())).yaw


def test_turn_left_yaws_ccw_positive():
    assert _yaw(SysCommand.S_Fly_TurnLeft, 15) == 15


def test_turn_right_yaws_cw_negative():
    # The bug: turn-right emitted +15 (CCW), the same as turn-left.
    assert _yaw(SysCommand.S_Fly_TurnRight, 15) == -15


def test_turn_left_and_right_are_opposite():
    assert _yaw(SysCommand.S_Fly_TurnLeft, 30) == -_yaw(SysCommand.S_Fly_TurnRight, 30)


def test_direction_follows_method_name_regardless_of_caller_sign():
    # DroneAPI.rotate passes a negative angle to single_fly_turnright; it must
    # still yaw right (CW/negative), not flip back to CCW.
    assert _yaw(SysCommand.S_Fly_TurnRight, -30) == -30
    assert _yaw(SysCommand.S_Fly_TurnLeft, -30) == 30


def test_retrace_inverse_cancels_the_turn():
    # A search turn (turn-right 15) and its retrace inverse (turn-left 15) must
    # sum to zero net yaw.
    assert _yaw(SysCommand.S_Fly_TurnRight, 15) + _yaw(SysCommand.S_Fly_TurnLeft, 15) == 0
