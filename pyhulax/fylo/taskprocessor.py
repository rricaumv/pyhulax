import inspect
import ctypes
import contextlib
from enum import Enum
from typing import TYPE_CHECKING
from ..fylo import mavlink
from itertools import groupby
from ..system.buffer import Buffer
from ..system import event, state
from ..fylo.commandprocessor import CommandProcessorFactory
from ..fylo.stateprocessor import StateProcessorFactory

if TYPE_CHECKING:
    from ..system.taskcontroller import TaskController


from ..system.datacenter import DataCenter, Device, PlaneType, UwbType, SysType
from ..system.command import SysCommand, Command
from ..system.state import *
from ..fylo.msganalyzer import MsgAnalyzer
from ..fylo.mavlink import *
from ..system.event import *
from socket import *
import time

UserTask = Enum(
    "UserTask",
    (
        "S_Fly_Takeoff",
        "S_Fly_Touchdown",
        "S_Fly_Forward",
        "S_Fly_Back",
        "S_Fly_Left",
        "S_Fly_Right",
        "S_Fly_Up",
        "S_Fly_Down",
        "S_Fly_unlock",
        "S_Fly_lock",
        "S_Fly_TurnLeft",
        "S_Fly_TurnRight",
        "S_Fly_TurnLeft360",
        "S_Fly_TurnRight360",
        "S_Fly_Bounce",
        "S_Fly_StraightFlight",
        "S_Fly_FlipForward",
        "S_Fly_FlipBack",
        "S_Fly_FlipLeft",
        "S_Fly_FlipRight",
        "S_Fly_Lamplight",
        "S_Fly_RadiusAround",
        "S_Fly_Linux_cmd",
        "S_Fly_CurvilinearFlight",
        "S_Fly_HoverFlight",
        "S_Fly_Barrier_aircraft",
        "S_Fly_Line_walking",
        "S_Fly_AiIdentifies",
        "S_Fly_Qr_tracking",
        "S_Fly_Qr_align",
        "S_Fly_ColorRecog",
        "S_Fly_Plane_time",
        "S_Fly_Enable_LED",
        "S_Fly_Disable_LED",
        "S_Fly_Cancel_RGB",
        "S_Fly_Vertical_Circle",
        "S_Fly_Set_Avoidance",
        "S_Fly_Get_Product_ID",
        "S_Fly_Set_Velocity",
        "S_Fly_Set_Yawrate",
        "S_Fly_Set_RGB_Brightness",
        "S_Fly_Enable_Battery_FS",
        "S_Fly_Disable_Battery_FS",
        "S_Fly_Set_Parameter",
        "S_Fly_Operate",
        "S_Fly_Set_Land_Speed",
        "S_Fly_Set_Video_Resolution",
        "S_Fly_Set_WiFi_Mode",
    ),
)

SysTask = Enum(
    "SysTask", ("P_State_GetHeartbeat", "P_Sys_SendTime", "P_Sys_CleanPlane")
)  


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
        raise TypeError("Only types can be raised (not instances)")
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


class Token:
    __instance = None

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance._flag = 1
            return cls.__instance
        else:
            return cls.__instance

    def __init__(self):
        if self._flag == 1:
            self._flag = 0
            self._token = 0
            self._lock = threading.Lock()

    def get_token(self):
        self._lock.acquire()
        self._token += 1
        if self._token > 255:
            self._token = 1

        self._lock.release()
        return self._token


def check_plane_status(task_controller, token, timeout_seconds=4):

    _tmp_time = time.time()

    while True:
        _data = task_controller._wait_state(SysState.P_Ack_GetFormation, timeout_seconds)
        if _data is None:
            if time.time() - _tmp_time > timeout_seconds:
                return 3
        else:
            # print(_data,'')
            if _data.get("result") == 255 and _data.get("token") == token:
                return 1
            elif (
                _data.get("result") == 241 or _data.get("result") == 242
            ) and _data.get("token") == token:
                return 0
            elif _data.get("result") == 240 and _data.get("token") == token:
                return 2


class TaskProcessor:
    user_task = None

    def __init__(self, task_controller: 'TaskController', data: dict):
        self._task_controller: 'TaskController' = task_controller
        self._data: dict = data
        self._droneId: int = 1
        # self._datacenter = DataCenter()
        self._returnValue: int = 1

    def work(self):
        pass

    def get_return_value(self):
        return self._returnValue


class Plane_Linux_cmd(TaskProcessor):
    user_task = UserTask.S_Fly_Linux_cmd

    def work(self):
        _plane_id = self._data["plane_id"]
        #  _token = self._data['_token']
        _token = Token().get_token()
        _utc = int(time.time())
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Linux_cmd,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "utc": _utc,
                    "cmd": self._data["cmd"],
                    "ack": self._data["ack"],
                    "type": self._data["type"],
                    "data": self._data["data"],
                    "reserve": self._data["reserve"],
                },
            )
        )

    #  self._task_controller._wait_state(SysState.P_Ack_GetFormation, 0.5)


class SFLamplight(TaskProcessor):
    "one plane lamplight TaskProcessor"

    user_task = UserTask.S_Fly_Lamplight

    ##
    def work(self):
        if "plane_id" not in self._data:
            print("SFS_Fly_lamplight parameter error")
            return

        _plane_id = self._data["plane_id"]
        R = self._data["r"]
        G = self._data["g"]
        B = self._data["b"]
        _mode = self._data["mode"]
        _time = self._data["time"]

        # _token = self._data['_token']
        _token = Token().get_token()

        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Lamplight,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "r": R,
                    "g": G,
                    "b": B,
                    "mode": _mode,
                    "time": _time,
                },
            )
        )
        time.sleep(_time)
        return True


class SFTakeoffTP(TaskProcessor):
    "one plane takeoff TaskProcessor"

    user_task = UserTask.S_Fly_Takeoff

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFTakeoffTP parameter error")
            return

        _plane_id = self._data["plane_id"]
        """
        if not self._datacenter.id_exist(Device.Plane, _plane_id):

            return
        """
        if "height" not in self._data:
            print("SFTakeoffTP parameter error")
            return
        _height = self._data["height"]
        if not isinstance(_height, int) or _height > 3000 or _height < 10:
            print(f"SFTakeoffTP parameter error: height={_height} (must be int 100-3000, firmware min=100cm)")
            return

        # _token = self._data['_token']
        _led = self._data["led"]
        _flags = self._data.get("flags", 0)  # TakeoffFlags bitmask
        _token = Token().get_token()

        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Takeoff,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "height": _height,
                    "led": _led,
                    "flags": _flags,
                },
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 10)
        if _ret <= 1:
            print("takeoff success")
            return True

        print("Takeoff not finish")

        return False


class SFTouchdownTP(TaskProcessor):
    user_task = UserTask.S_Fly_Touchdown

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFTouchdownTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        """
        if not self._datacenter.id_exist(Device.Plane, _plane_id):

            return
        """
        _token = Token().get_token()
        # _token = self._data['_token']
        _led = self._data["led"]
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Touchdown,
                    {"plane_id": _plane_id, "token": _token, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("touchdown success")
                return True
            else:
                continue
        print("Touchdown not finish")
        return False


# pass
class SFForwardTP(TaskProcessor):
    user_task = UserTask.S_Fly_Forward

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFForwardTP parameter error")
            return
        _plane_id = self._data["plane_id"]

        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        
        #    return

        if "distance" not in self._data:
            print("SFForwardTP parameter error")
            return
        _distance = self._data["distance"]
        if not isinstance(_distance, int) or _distance > 500 or _distance < 5:
            print("SFForwardTP parameter error")
            return
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Forward,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "distance": _distance,
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("Forward finish")
                return True
        print("Forward not finish")
        return False


# pass
class SFBackTP(TaskProcessor):
    user_task = UserTask.S_Fly_Back

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFBackTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFBackTP plane is not ready')
        #    return

        if "distance" not in self._data:
            print("SFBackTP parameter error")
            return
        _distance = self._data["distance"]
        if not isinstance(_distance, int) or _distance > 500 or _distance < 5:
            print("SFBackTP parameter error")
            return

        # _token = self._data['_token']
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Back,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "distance": -(_distance),
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("Back finish")
                return True
        print("Back not finish")
        return False


class SFLeftTP(TaskProcessor):
    user_task = UserTask.S_Fly_Left

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFLeftTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFLeftTP plane is not ready')
        #    return

        if "distance" not in self._data:
            print("SFLeftTP parameter error")
            return
        _distance = self._data["distance"]
        if not isinstance(_distance, int) or _distance > 500 or _distance < 5:
            print("SFLeftTP parameter error")
            return

        # _token = self._data['_token']
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Left,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "distance": _distance,
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("Left finish")
                return True
        print("Left not finish")
        return False





class SFRightTP(TaskProcessor):
    user_task = UserTask.S_Fly_Right

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFRightTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFRightTP plane is not ready')
        #    return

        if "distance" not in self._data:
            print("SFRightTP parameter error")
            return
        _distance = self._data["distance"]
        if not isinstance(_distance, int) or _distance > 500 or _distance < 5:
            print("SFRightTP parameter error")
            return

        # _token = self._data['_token']
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Right,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "distance": -(_distance),
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("Right finish")
                return True
        print("Right not finish")
        return False


class SFUpTP(TaskProcessor):
    user_task = UserTask.S_Fly_Up

    def work(self):
        _resend_num = 2
        if "plane_id" not in self._data:
            print("SFUpTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFUpTP plane is not ready')
        #    return

        if "height" not in self._data:
            print("SFUpTP parameter error")
            return
        _height = self._data["height"]
        if not isinstance(_height, int) or _height > 500 or _height < 5:
            print("SFUpTP parameter error")
            return

        # _token = self._data['_token']
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Up,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "height": _height,
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("SFUpTP finish")
                return True
        print("SFUpTP not finish")
        return False


class SFDownTP(TaskProcessor):
    user_task = UserTask.S_Fly_Down

    def work(self):
        if "plane_id" not in self._data:
            print("SFDownTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFDownTP plane is not ready')
        #    return

        if "height" not in self._data:
            print("SFDownTP parameter error")
            return
        _height = self._data["height"]
        if not isinstance(_height, int) or _height > 500 or _height < 5:
            print("SFDownTP parameter error")
            return

        # _token = self._data['_token']
        _token = Token().get_token()
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        _resend_num = 2
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Down,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "height": -(_height),
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("SFDownTP finish")
                return True
        print("SFDownTP not finish")
        return False


class SFTurnLeftTP(TaskProcessor):
    user_task = UserTask.S_Fly_TurnLeft

    def work(self):
        if "plane_id" not in self._data:
            print("SFTurnLeftTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFTurnLeftTP plane is not ready')
        #    return

        if "angle" not in self._data:
            print("SFTurnLeftTP parameter error")
            return
        _angle = self._data["angle"]
        if not isinstance(_angle, int) or _angle > 360 or _angle < 0:
            print("SFTurnLeftTP parameter error")
            return

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_TurnLeft,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "angle": _angle,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("SFTurnLeftTP finish")
                return True
        print("SFTurnLeftTP not finish")
        return False


class SFTurnRightTP(TaskProcessor):
    user_task = UserTask.S_Fly_TurnRight

    def work(self):
        if "plane_id" not in self._data:
            print("SFTurnRightTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFTurnRightTP plane is not ready')
        #    return

        if "angle" not in self._data:
            print("SFTurnRightTP parameter error")
            return
        _angle = self._data["angle"]
        # if not isinstance(_angle, int) or _angle > 360 or _angle < 0:
        #     print('SFTurnRightTP parameter error')
        

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_TurnRight,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "angle": _angle,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                print("SFTurnRightTP finish")
                return True
        print("SFTurnRightTP not finish")
        return False


class SFTurnLeft360TP(TaskProcessor):
    user_task = UserTask.S_Fly_TurnLeft360

    def work(self):
        if "plane_id" not in self._data:
            print("SFTurnLeft360TP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFTurnLeft360TP plane is not ready')
        
        
        
        #     print('SFTurnLeft360TP parameter error')
        #     return
        _num = self._data["num"]
        # if not isinstance(_num, int) or _num > 10 or _num < 0:
        #     print('SFTurnLeft360TP parameter error')
        

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_TurnLeft360,
                    {"plane_id": _plane_id, "token": _token, "num": _num, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token, abs(_num) * 10)
            if _ret <= 1:
                print("SFTurnLeft360TP finish")
                return True
        print("SFTurnLeft360TP not finish")
        return False


class SFTurnRight360TP(TaskProcessor):
    user_task = UserTask.S_Fly_TurnRight360

    def work(self):
        if "plane_id" not in self._data:
            print("SFTurnRight360TP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFTurnRight360TP plane is not ready')
        #    return

        if "num" not in self._data:
            print("SFTurnRight360TP parameter error")
            return
        _num = self._data["num"]
        if not isinstance(_num, int) or _num > 10 or _num < 0:
            print("SFTurnRight360TP parameter error")
            return

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_TurnRight360,
                    {"plane_id": _plane_id, "token": _token, "num": _num, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFTurnRight360TP not finish")
        return False


class SFBounceTP(TaskProcessor):
    user_task = UserTask.S_Fly_Bounce

    def work(self):

        _plane_id = self._data["plane_id"]
        _height = self._data["height"]
        _frequency = self._data["frequency"]
        if not isinstance(_height, int) or _height > 500 or _height < 5:
            print("SFBounceTP parameter error")
            return

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num * _frequency):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Bounce,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "height": _height,
                        "frequency": _frequency,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token, _frequency * 3)
            if _ret <= 1:
                return True
        print("SFBounceTP not finish")
        return False





class SFStraightFlightTP(TaskProcessor):
    user_task = UserTask.S_Fly_StraightFlight

    def work(self):
        if "plane_id" not in self._data:
            print("SFStraightFlightTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFStraightFlightTP plane is not ready')
        #    return

        if "x" not in self._data or "y" not in self._data or "z" not in self._data:
            print("SFStraightFlightTP parameter error")
            return
        _x = self._data["x"]
        _y = self._data["y"]
        _z = self._data["z"]

        # _token = self._data['_token']
        _led = self._data["led"]
        _speed = self._data.get("speed", 100)
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_StraightFlight,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "x": _x,
                        "y": _y,
                        "z": _z,
                        "led": _led,
                        "speed": _speed,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFStraightFlightTP not finish")
        return False


class SFFlipForwardTP(TaskProcessor):
    user_task = UserTask.S_Fly_FlipForward

    def work(self):
        if "plane_id" not in self._data:
            print("SFFlipForwardTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFFlipForwardTP plane is not ready')
        

        # _token = self._data['_token']
        _direction = self._data["direction"]
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_FlipForward,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "direction": _direction,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFFlipForwardTP not finish")
        return False


class SFFlipBackTP(TaskProcessor):
    user_task = UserTask.S_Fly_FlipBack

    def work(self):
        if "plane_id" not in self._data:
            print("SFFlipBackTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFFlipBackTP plane is not ready')
        

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_FlipBack,
                    {"plane_id": _plane_id, "token": _token, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFFlipBackTP not finish")
        return False


class SFFlipLeftTP(TaskProcessor):
    user_task = UserTask.S_Fly_FlipLeft

    def work(self):
        if "plane_id" not in self._data:
            print("SFFlipLeftTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFFlipLeftTP plane is not ready')
        

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_FlipLeft,
                    {"plane_id": _plane_id, "token": _token, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFFlipLeftTP not finish")
        return False


class SFFlipRightTP(TaskProcessor):
    user_task = UserTask.S_Fly_FlipRight

    def work(self):
        if "plane_id" not in self._data:
            print("SFFlipRightTP parameter error")
            return
        _plane_id = self._data["plane_id"]
        # if not self._datacenter.id_exist(Device.Plane, _plane_id):
        #    print('SFFlipRightTP plane is not ready')
        

        # _token = self._data['_token']
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_FlipRight,
                    {"plane_id": _plane_id, "token": _token, "led": _led},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token)
            if _ret <= 1:
                return True
        print("SFFlipRightTP not finish")
        return False


class SFCurvilinearFlight(TaskProcessor):
    user_task = UserTask.S_Fly_CurvilinearFlight

    def work(self):
        if "plane_id" not in self._data:
            print("SFCurvilinearFlight parameter error")
            return
        _plane_id = self._data["plane_id"]
        # _token = self._data['_token']
        _x = self._data["x"]
        _y = self._data["y"]
        _z = self._data["z"]
        _led = self._data["led"]
        _direction = self._data["direction"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_CurvilinearFlight,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "x": _x,
                        "y": _y,
                        "led": _led,
                        "z": _z,
                        "direction": _direction,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token, 8)
            if _ret <= 1:
                return True
        print("SFCurvilinearFlight not finish")
        return False


class SFHoverFlight(TaskProcessor):
    user_task = UserTask.S_Fly_HoverFlight

    def work(self):
        _plane_id = self._data["plane_id"]
        # _token = self._data['_token']
        _time = self._data["time"]
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_HoverFlight,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "time": _time,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token, _time + 1)
            if _ret <= 1:
                return True
        print("SFHoverFlight not finish")
        return False


class SFBarrier_aircraft(TaskProcessor):
    user_task = UserTask.S_Fly_Barrier_aircraft

    def work(self):
        _plane_id = self._data["plane_id"]
        _mode = self._data["mode"]
        _token = self._data["_token"]
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Barrier_aircraft,
                {"plane_id": _plane_id, "token": _token, "mode": _mode},
            )
        )


class SFLine_walking(TaskProcessor):
    user_task = UserTask.S_Fly_Line_walking

    def work(self):
        _fun_id = self._data["fun_id"]
        _dist = self._data["dist"]
        _tv = self._data["tv"]
        _way_color = self._data["way_color"]
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Line_walking,
                {"fun_id": _fun_id, "dist": _dist, "tv": _tv, "way_color": _way_color},
            )
        )


class SFAiIdentifies(TaskProcessor):
    user_task = UserTask.S_Fly_AiIdentifies

    def work(self):
        _mode = self._data["mode"]
        self._task_controller._send_command(
            Command(SysCommand.S_Fly_AiIdentifies, {"mode": _mode})
        )


class SFQr_code_tracking(TaskProcessor):
    user_task = UserTask.S_Fly_Qr_tracking

    def work(self):
        _mode = self._data["mode"]
        _type = self._data["type"]

        _time_duration = self._data["time_duration"]
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Qr_tracking,
                {"mode": _mode, "type": _type, "time_duration": _time_duration},
            )
        )


class SFQr_code_aligns(TaskProcessor):
    user_task = UserTask.S_Fly_Qr_align

    def work(self):
        _mode = self._data["mode"]
        _time_duration = self._data["time_duration"]
        _search_radius = self._data["search_radius"]
        _qr_id = self._data["qr_id"]
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Qr_align,
                {
                    "mode": _mode,
                    "search_radius": _search_radius,
                    "time_duration": _time_duration,
                    "qr_id": _qr_id,
                },
            )
        )


class SF_ColorRecog(TaskProcessor):
    user_task = UserTask.S_Fly_ColorRecog

    def work(self):
        _Mode = self._data["Mode"]
        self._task_controller._send_command(
            Command(SysCommand.S_Fly_ColorRecog, {"Mode": _Mode})
        )
        return


class SF_unlock(TaskProcessor):
    user_task = UserTask.S_Fly_unlock

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = self._data["_token"]

        self._task_controller._send_command(
            Command(SysCommand.S_Fly_unlock, {"plane_id": _plane_id, "token": _token})
        )

        # data = self._task_controller._wait_state(SysState.P_Ack_GetFormation, 20)





class SF_lock(TaskProcessor):
    user_task = UserTask.S_Fly_lock

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = self._data["_token"]

        self._task_controller._send_command(
            Command(SysCommand.S_Fly_lock, {"plane_id": _plane_id, "token": _token})
        )

        return


class SFCircumvolant(TaskProcessor):
    user_task = UserTask.S_Fly_RadiusAround

    def work(self):
        _plane_id = self._data["plane_id"]
        # _token = self._data['_token']
        _radius = self._data["radius"]
        _led = self._data["led"]
        _resend_num = 2
        _token = Token().get_token()
        for i in range(_resend_num):

            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_RadiusAround,
                    {
                        "plane_id": _plane_id,
                        "token": _token,
                        "radius": _radius,
                        "led": _led,
                    },
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(
                self._task_controller, _token, abs(_radius / 2 + 5)
            )
            if _ret <= 1:
                return True
        print("SFCircumvolant not finish")
        return False


class SF_Plane_time(TaskProcessor):
    user_task = UserTask.S_Fly_Plane_time

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Plane_time, {"plane_id": _plane_id, "token": _token}
            )
        )
        return True


class SF_Enable_LED(TaskProcessor):
    """Enable LED task - formation cmd 0x0C (12)"""
    user_task = UserTask.S_Fly_Enable_LED

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Enable_LED, {"plane_id": _plane_id, "token": _token}
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Disable_LED(TaskProcessor):
    """Disable LED task - formation cmd 0x0D (13)"""
    user_task = UserTask.S_Fly_Disable_LED

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Disable_LED, {"plane_id": _plane_id, "token": _token}
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Cancel_RGB(TaskProcessor):
    """Cancel RGB animation task - formation cmd 0x1B (27)"""
    user_task = UserTask.S_Fly_Cancel_RGB

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Cancel_RGB, {"plane_id": _plane_id, "token": _token}
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Vertical_Circle(TaskProcessor):
    """Vertical circle maneuver - formation cmd 0x27 (39)

    Requires altitude >= 0.35m
    """
    user_task = UserTask.S_Fly_Vertical_Circle

    def work(self):
        _plane_id = self._data["plane_id"]
        _radius = self._data["radius"]
        _token = Token().get_token()
        _resend_num = 2
        for i in range(_resend_num):
            self._task_controller._send_command(
                Command(
                    SysCommand.S_Fly_Vertical_Circle,
                    {"plane_id": _plane_id, "token": _token, "radius": _radius},
                )
            )
            if not self._data.get("blocking", True):
                return True
            _ret = check_plane_status(self._task_controller, _token, 10)
            if _ret <= 1:
                return True
        return False


class SF_Set_Avoidance(TaskProcessor):
    """Set avoidance with direction - formation cmd 0x2A (42)"""
    user_task = UserTask.S_Fly_Set_Avoidance

    def work(self):
        _plane_id = self._data["plane_id"]
        _direction = self._data.get("direction", 0)
        _barrier_mask = self._data.get("barrier_mask", 0x3F)
        _x = self._data.get("x", 0)
        _y = self._data.get("y", 0)
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Avoidance,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "direction": _direction,
                    "barrier_mask": _barrier_mask,
                    "x": _x,
                    "y": _y,
                },
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Get_Product_ID(TaskProcessor):
    """Get product ID/autopilot version - formation cmd 0x2C (44)"""
    user_task = UserTask.S_Fly_Get_Product_ID

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Get_Product_ID,
                {"plane_id": _plane_id, "token": _token},
            )
        )
        # No ACK expected for this command, it triggers autopilot version response
        return True


class SF_Set_Velocity(TaskProcessor):
    """Set velocity level - formation cmd 0x30 (48)

    Level is velocity in cm/s (0-300): 100=1.0m/s, 200=2.0m/s, 300=3.0m/s
    """
    user_task = UserTask.S_Fly_Set_Velocity

    def work(self):
        _plane_id = self._data["plane_id"]
        _level = self._data["level"]
        _horizontal_vel = self._data.get("horizontal_vel", 0)
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Velocity,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "level": _level,
                    "horizontal_vel": _horizontal_vel,
                },
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Set_Yawrate(TaskProcessor):
    """Set yaw rate level - formation cmd 0x31 (49)"""
    user_task = UserTask.S_Fly_Set_Yawrate

    def work(self):
        _plane_id = self._data["plane_id"]
        _level = self._data["level"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Yawrate,
                {"plane_id": _plane_id, "token": _token, "level": _level},
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Set_RGB_Brightness(TaskProcessor):
    """Set RGB brightness - formation cmd 0x32 (50)"""
    user_task = UserTask.S_Fly_Set_RGB_Brightness

    def work(self):
        _plane_id = self._data["plane_id"]
        _brightness = self._data["brightness"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_RGB_Brightness,
                {"plane_id": _plane_id, "token": _token, "brightness": _brightness},
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Enable_Battery_FS(TaskProcessor):
    """Enable battery failsafe - formation cmd 0x35 (53)"""
    user_task = UserTask.S_Fly_Enable_Battery_FS

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Enable_Battery_FS,
                {"plane_id": _plane_id, "token": _token},
            )
        )
        # No ACK for this command
        return True


class SF_Disable_Battery_FS(TaskProcessor):
    """Disable battery failsafe - formation cmd 0x36 (54)"""
    user_task = UserTask.S_Fly_Disable_Battery_FS

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Disable_Battery_FS,
                {"plane_id": _plane_id, "token": _token},
            )
        )
        # No ACK for this command
        return True


class SF_Set_Parameter(TaskProcessor):
    """Set multiple parameters - formation cmd 0x37 (55)"""
    user_task = UserTask.S_Fly_Set_Parameter

    def work(self):
        _plane_id = self._data["plane_id"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Parameter,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "velocity": self._data.get("velocity", 0),
                    "yaw_rate": self._data.get("yaw_rate", 0),
                    "brightness": self._data.get("brightness", 0),
                    "avoidance": self._data.get("avoidance", False),
                    "battery_failsafe": self._data.get("battery_failsafe", False),
                    "fast_land": self._data.get("fast_land", False),
                },
            )
        )
        if not self._data.get("blocking", True):
            return True
        _ret = check_plane_status(self._task_controller, _token, 2)
        if _ret <= 1:
            return True
        return False


class SF_Operate(TaskProcessor):
    """Set formation operate status - formation cmd 0x3B (59)"""
    user_task = UserTask.S_Fly_Operate

    def work(self):
        _plane_id = self._data["plane_id"]
        _status = self._data["status"]
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Operate,
                {"plane_id": _plane_id, "token": _token, "status": _status},
            )
        )
        # No ACK for this command
        return True


class SF_Set_Land_Speed(TaskProcessor):
    """Set land speed - formation cmd 0x3C (60)

    fast=False: slow landing, fast=True: fast landing
    """
    user_task = UserTask.S_Fly_Set_Land_Speed

    def work(self):
        _plane_id = self._data["plane_id"]
        _fast = self._data.get("fast", False)
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Land_Speed,
                {"plane_id": _plane_id, "token": _token, "fast": _fast},
            )
        )
        # No ACK for this command
        return True


class SF_Set_Video_Resolution(TaskProcessor):
    """Set video resolution - plane cmd 0x16 (22)

    resolution: 0=HIGH (1080p), 1=MEDIUM (720p), 2=LOW (program/AI mode)

    Lower resolution = less encoder CPU = potentially better QR rate during RTP.
    Firmware handler: HandleMsgSelectRecordResolution @ avmanager:0x0001df7c
    """
    user_task = UserTask.S_Fly_Set_Video_Resolution

    def work(self):
        _plane_id = self._data["plane_id"]
        _resolution = self._data.get("resolution", 0)
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_Video_Resolution,
                {"plane_id": _plane_id, "token": _token, "resolution": _resolution},
            )
        )
        # Plane command - returns ACK via plane_ack message
        return True


class SF_Set_WiFi_Mode(TaskProcessor):
    """Set WiFi mode - plane cmd 0x04 (4)

    WiFi mode values:

    - 0: Switch to 2.4GHz band
    - 1: Switch to 5GHz band
    - 2: Switch to AP mode
    - 3: Low WiFi power
    - 4: High WiFi power
    - 5: WiFi broadcast ON
    - 6: WiFi broadcast OFF
    - 7: Manual channel mode (requires channel_id)
    - 8: Auto channel mode
    - 9: Get channel strength

    Firmware handler: HandleMsgWifiMode @ avmanager
    """
    user_task = UserTask.S_Fly_Set_WiFi_Mode

    def work(self):
        _plane_id = self._data["plane_id"]
        _wifi_mode = self._data.get("wifi_mode", 0)
        _channel_id = self._data.get("channel_id", 0)
        _token = Token().get_token()
        self._task_controller._send_command(
            Command(
                SysCommand.S_Fly_Set_WiFi_Mode,
                {
                    "plane_id": _plane_id,
                    "token": _token,
                    "wifi_mode": _wifi_mode,
                    "channel_id": _channel_id,
                },
            )
        )
        # Plane command - returns ACK via plane_ack message
        return True


task_processor_list = [
    SFTakeoffTP,
    SFTouchdownTP,
    SFForwardTP,
    SFBackTP,
    SFLeftTP,
    SFRightTP,
    SFUpTP,
    SFDownTP,
    SFTurnLeftTP,
    SFTurnRightTP,
    SFTurnLeft360TP,
    SFTurnRight360TP,
    SFBounceTP,
    SFStraightFlightTP,
    SFFlipForwardTP,
    SFFlipBackTP,
    SFFlipLeftTP,
    SFFlipRightTP,
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


class TaskProcessorFactory:
    def get_task_processor(task_controller, utask, data):
        for cl in task_processor_list:
            if cl.user_task == utask:
                return cl(task_controller, data)
        return None
