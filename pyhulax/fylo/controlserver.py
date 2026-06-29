"""
Docstring for pyhulax.fylo.controlserver
"""

import os
import socket
import time
import enum
import threading
from collections.abc import Callable

import psutil
from pyhulax.config import DroneConfig, resolve_config
from ..system.taskcontroller import TaskController, UserTask, SysTask
from ..system.datacenter import *

from ..system.state import SysState
from . import config as fylo_config

################################tcp###################################
ConnectType = enum.Enum("ConnectType", ("Serial", "Network", "Tcpwork"))
ManualFlyFrameCallback = Callable[[float, float, float, float, int, bool], None]


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
        self._token += 1
        if self._token > 255:
            self._token = 1

        return self._token


def get_wifi_ip():
    #
    info = psutil.net_if_addrs()

    # WiFi
    for interface_name, addresses in info.items():
        # IPv4
        for addr in addresses:
            if addr.family == socket.AF_INET:
                # 'Wi-Fi', 'WLAN',  'wlan'
                if (
                        "wi-fi" in interface_name.lower()
                        or "wlp2s0" in interface_name.lower()
                ):
                    return interface_name, addr.address

    return None, None


class Controlserver:
    def __init__(
        self,
        runtime_config: DroneConfig | None = None,
        drone_id: int | None = None,
    ):
        self._config = resolve_config(runtime_config)
        self._server_ip = self._config.network.drone_ip
        self._connect_status = 0
        self._datacenter: DataCenter = DataCenter()
        self._taskcontroller: TaskController | None = None
        # Optional explicit per-connection drone id. When omitted the id is
        # discovered from this connection's own telemetry after connect().
        self._drone_id = drone_id

        fylo_config.apply_runtime_config(self._config)

        # Auto-detect bind_client from local IP on drone network (192.168.100.x)
        if fylo_config.bind_client == 255:
            detected = self._detect_bind_client()
            if detected is not None:
                fylo_config.bind_client = detected

    def _detect_bind_client(self, subnet: str | None = None) -> int | None:
        """Detect bind_client from local IP on the drone network."""
        if subnet is None:
            subnet = ".".join(self._config.network.drone_ip.split(".")[:-1])
        try:
            for _, addresses in psutil.net_if_addrs().items():
                for addr in addresses:
                    if addr.family == socket.AF_INET:
                        ip = addr.address
                        if ip.startswith(subnet + "."):
                            return int(ip.split(".")[-1])
        except Exception:
            pass
        return None

    # =============================================Drone Identity======================================================================#

    def _resolve_drone_id(self) -> int | None:
        """Resolve this connection's drone id.

        Priority: explicit id passed to this Controlserver > id discovered by
        the TaskController from this connection's telemetry > legacy global.
        """
        if self._drone_id is not None:
            return self._drone_id
        if self._taskcontroller is not None:
            discovered = self._taskcontroller.get_drone_id()
            if discovered is not None:
                return discovered
        return fylo_config.drone_id

    def _resolve_command_id(self) -> int:
        """Drone id to stamp into outgoing command payloads (never None)."""
        resolved = self._resolve_drone_id()
        return resolved if resolved is not None else 1

    def _get_plane_data(self, datatype: str):
        """Read per-drone telemetry, falling back to the legacy shared slot.

        Telemetry is mirrored by the TaskController into a DataCenter slot keyed
        by the real drone id. Before that id is known (or for legacy single-drone
        callers) we fall back to slot ``id=0`` so behaviour is unchanged.
        """
        resolved = self._resolve_drone_id()
        if resolved is not None:
            data = self._datacenter.get_data("Plane", datatype, resolved)
            if data is not None:
                return data
        return self._datacenter.get_data("Plane", datatype)

    def get_plane_data(self, datatype: str = "flight_data"):
        """Public accessor for per-drone telemetry (e.g. flight_data, heartbeat)."""
        return self._get_plane_data(datatype)

    # =============================================System Config======================================================================#

    def connect(
        self,
        server_ip,
        enable_file_logging: bool = True,
        log_dir: str = "logs",
        connect_timeout: float | None = None,
    ):
        self._connect_status = 1
        if server_ip is None:
            server_ip = self._config.network.drone_ip

        self._server_ip = server_ip
        self._taskcontroller = TaskController(
            server_ip,
            runtime_config=self._config,
            enable_file_logging=enable_file_logging,
            log_dir=log_dir,
            drone_id=self._drone_id,
        )
        print(f"connect to {server_ip}")

        # Start broadcasting APP_HEARTBEAT before waiting. Many hula drones only
        # begin streaming their own REPORT_STATS/heartbeat once the ground
        # station announces itself, so sending this first maximises the chance
        # of receiving telemetry during the connect window.
        self._taskcontroller.udp_heartbeat_send_thread()

        # Poll for the drone's heartbeat instead of a single fixed-delay check.
        # Slow links/drones get the full timeout; fast ones return as soon as
        # the first heartbeat lands.
        if connect_timeout is None:
            connect_timeout = max(5.0, self._config.timeouts.tcp_connect_timeout_sec)
        deadline = time.time() + connect_timeout
        data = None
        while time.time() < deadline:
            data = self._get_plane_data("heartbeat")
            if data is not None:
                break
            time.sleep(0.2)

        if data is None:
            print(
                f"connect error: no heartbeat from {server_ip} within "
                f"{connect_timeout:.0f}s"
            )
            self._connect_status = 0
            try:
                self._taskcontroller.stop_all_task()
            except Exception:
                pass
            return False  # TODO : RESTORE
        print("connect wifi")
        self._taskcontroller.create_task(
            UserTask.S_Fly_Plane_time, {"plane_id": self._resolve_command_id()}
        )
        return True

    def disconnect(self):
        if self._connect_status == 1:
            self._taskcontroller.stop_all_task()
            # self._communication_controller.disconnect()
            # self._datacenter.empty_datacenter()
            self._connect_status = 0
            # Give threads time to exit cleanly
            import time
            (time.
             sleep(0.5))

    def __del__(self):
        """Cleanup when object is garbage collected"""
        try:
            self.disconnect()
        except:
            pass

    # ! MISSING: Line (104)
    # =============================================RealTime Control===================================================================#
    #
    # def planeDate_decorator(func):
    #     def wrapper(self, *args, **kw):
    #         _token = Token().get_token()
    #         print(func, *args, **kw)
    #         func(self, *args, _token, **kw)

    #         data = self._taskcontroller._wait_state(SysState.P_Ack_GetFormation, 20)

    #         if data:
    #             # If result is 255 and token matches, return True
    #             if data.get("result") == 255 and data.get("token") == _token:
    #                 return True
    #             # If result is 240 and it's the first retry, retry the wrapper function

    #             else:
    #                 return False
    #         else:
    #             return False
    #         # If data is empty or other cases, return False

    #     return wrapper

    # 8.Linux
    def Plane_Linux_cmd(self, cmd, ack, mode, data, reserve):
        # Use the per-connection drone id (discovered or explicitly configured)
        plane_id = self._resolve_command_id()

        self._taskcontroller.create_task(
            UserTask.S_Fly_Linux_cmd,
            {
                "_token": 0,
                "plane_id": plane_id,
                "cmd": cmd,
                "ack": ack,
                "type": mode,
                "data": data,
                "reserve": reserve,
            },
        )

    def Plane_getBarrier(self):

        data = self._get_plane_data("flight_data")
        _barrierList = {
            "forward": False,
            "back": False,
            "left": False,
            "right": False,
        }
        if data == None:
            return _barrierList

        barrier = data.barrier
        # m_DownBarrier = (barrier & 16) == 16
        _barrierList["forward"] = (barrier & 1) == 1
        _barrierList["back"] = (barrier & 2) == 2
        _barrierList["left"] = (barrier & 4) == 4
        _barrierList["right"] = (barrier & 8) == 8
        return _barrierList

    def get_battery(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return 0
        return data.battery_volumn

    def get_coordinate(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return [0, 0, 0]
        return [int(data.x), int(data.y), int(data.z)]

    def get_laser_receiving(self):

        data = self._taskcontroller._wait_state(SysState.P_State_Photoresponse, 1)
        if data == None:
            return False
        return data.get("cmd") == 7

    def get_yaw(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return [0, 0, 0]
        return [int(data.yaw / 100), int(data.pitch / 100), int(data.roll / 100)]

    def get_accelerated_speed(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return [0, 0, 0]
        return [int(data.accx), int(data.accy), int(data.accz)]

    def get_plane_speed(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return [0, 0, 0]
        return [int(data.vel_x), int(data.vel_y), int(data.vel_z)]

    def get_plane_distance(self):
        data = self._get_plane_data("flight_data")
        if data == None:
            return 0
        return int(data.distance)

    def get_plane_id(self):
        return self._resolve_drone_id()

    def single_fly_lamplight(self, r, g, b, duration, mode, token=0):
        # print(self._taskcontroller._datacenter.get_data(Device.Plane,SysState.P_State_GetHeartbeat,1),8888888888888)
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Lamplight,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "r": r,
                "g": g,
                "b": b,
                "mode": mode,
                "time": duration,
            },
        )

    # @planeDate_decorator
    def single_fly_takeoff(
        self,
        led: int,
        height: int = 100,
        flags: int = 0,
        token: int = 0,
        blocking: bool = True,
    ):
        """Takeoff to specified height.

        Args:
            led: LED color param4 value
            height: Target height in cm (50-3000)
            flags: TakeoffFlags bitmask (0=normal, 1=reset_yaw, 2=with_load)
            token: Command token (0=auto)
            blocking: Wait for completion
        """
        data = self._get_plane_data("heartbeat")
        if data != None and data.drone_status == 2:
            return self._taskcontroller.create_task(
                UserTask.S_Fly_Takeoff,
                {
                    "_token": token,
                    "plane_id": self._resolve_command_id(),
                    "height": height,
                    "led": led,
                    "flags": flags,
                    "blocking": blocking,
                },
            )
        else:
            print("plane-status error")
            os._exit(0)

    # @planeDate_decorator
    def single_fly_touchdown(self, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Touchdown, {"_token": token, "plane_id": self._resolve_command_id(), "led": led, "blocking": blocking}
        )

    # @planeDate_decorator
    def single_fly_forward(self, distance, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Forward,
            {"_token": token, "plane_id": self._resolve_command_id(), "distance": distance, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_back(self, distance, led, token=0, blocking=True, speed=100):

        return self._taskcontroller.create_task(
            UserTask.S_Fly_Back,
            {"_token": token, "plane_id": self._resolve_command_id(), "distance": distance, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_left(self, distance, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Left,
            {"_token": token, "plane_id": self._resolve_command_id(), "distance": distance, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_right(self, distance, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Right,
            {"_token": token, "plane_id": self._resolve_command_id(), "distance": distance, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_up(self, height, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Up,
            {"_token": token, "plane_id": self._resolve_command_id(), "height": height, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_down(self, height, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Down,
            {"_token": token, "plane_id": self._resolve_command_id(), "height": height, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_turnleft(self, angle, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_TurnLeft,
            {"_token": token, "plane_id": self._resolve_command_id(), "angle": angle, "led": led, "blocking": blocking},
        )

    # @planeDate_decorator
    def single_fly_turnright(self, angle, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_TurnRight,
            {"_token": token, "plane_id": self._resolve_command_id(), "angle": angle, "led": led, "blocking": blocking},
        )

    # @planeDate_decorator
    def single_fly_radius_around(self, radius, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_RadiusAround,
            {"_token": token, "plane_id": self._resolve_command_id(), "radius": radius, "led": led, "blocking": blocking},
        )

    # @planeDate_decorator
    def single_fly_curvilinearFlight(self, direction, x, y, z, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_CurvilinearFlight,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "x": x,
                "y": y,
                "z": z,
                "direction": direction,
                "led": led,
                "blocking": blocking,
            },
        )

    # @planeDate_decorator
    def single_fly_autogyration360(self, num, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_TurnLeft360,
            {"_token": token, "plane_id": self._resolve_command_id(), "num": num, "led": led, "blocking": blocking},
        )

    # def single_fly_turnright360(self,   num, token = 0):
    # ! MISSING: Line (264)

    # @planeDate_decorator
    def single_fly_hover_flight(self, duration, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_HoverFlight,
            {"_token": token, "plane_id": self._resolve_command_id(), "time": duration, "led": led, "blocking": blocking},
        )

    # @planeDate_decorator
    def single_fly_bounce(self, frequency, height, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Bounce,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "height": height,
                "frequency": frequency,
                "led": led,
                "blocking": blocking,
            },
        )

    # @planeDate_decorator
    def single_fly_straight_flight(self, x, y, z, led, token=0, blocking=True, speed=100):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_StraightFlight,
            {"_token": token, "plane_id": self._resolve_command_id(), "x": x, "y": y, "z": z, "led": led, "blocking": blocking, "speed": speed},
        )

    # @planeDate_decorator
    def single_fly_barrier_aircraft(self, mode, token=0):
        self._taskcontroller.create_task(
            UserTask.S_Fly_Barrier_aircraft,
            {"_token": token, "plane_id": self._resolve_command_id(), "mode": mode},
        )

    # @planeDate_decorator
    def single_fly_somersault(self, direction, led, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_FlipForward,
            {"_token": token, "plane_id": self._resolve_command_id(), "direction": direction, "led": led, "blocking": blocking},
        )

    def single_fly_flip_back(self, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_FlipBack, {"_token": token, "plane_id": self._resolve_command_id(), "led": 0, "blocking": blocking}
        )

    def single_fly_flip_left(self, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_FlipLeft, {"_token": token, "plane_id": self._resolve_command_id(), "led": 0, "blocking": blocking}
        )

    def single_fly_flip_right(self, token=0, blocking=True):
        return self._taskcontroller.create_task(
            UserTask.S_Fly_FlipRight, {"_token": token, "plane_id": self._resolve_command_id(), "led": 0, "blocking": blocking}
        )

    def single_fly_flip_rtp(self):
        self._taskcontroller.udp_rtp_udp_recive_thread()

    def single_fly_Line_walking(self, fun_id, dist, tv, way_color):
        self._taskcontroller.create_task(
            UserTask.S_Fly_Line_walking,
            {"fun_id": fun_id, "dist": dist, "tv": tv, "way_color": way_color},
        )
        while True:

            data = self._taskcontroller._wait_state(SysState.P_State_WALKING, 0.5)
            if data == None:
                continue
            time.sleep(0.5)
            return data

    def single_fly_AiIdentifies(self, mode):
        self._taskcontroller.create_task(UserTask.S_Fly_AiIdentifies, {"mode": mode})
        _data = {
            "mode": 0,
            "type": 0,
            "x": 0,
            "y": 0,
            "z": 0,
            "angle": 0,
            "result": False,
        }
        while True:
            data = self._taskcontroller._wait_state(SysState.P_State_CAMERA, 0.5)

            if data == None:
                continue
            _data["mode"] = data["mode"]
            _data["type"] = data["type"]
            #   _data['time_duration'] = data['time_duration']
            _data["x"] = int(data["x"] * 100)
            _data["y"] = int(data["y"] * 100)
            _data["z"] = int(data["z"] * 100)
            _data["angle"] = int(data["angle"])
            _data["result"] = data["result"] == 1
            time.sleep(0.5)
            return _data

    def single_fly_Qrcode_tracking(self, mode, tracking_type, time_duration):
        self._taskcontroller.create_task(
            UserTask.S_Fly_Qr_tracking,
            {"mode": mode, "type": tracking_type, "time_duration": time_duration},
        )
        _data = {
            "mode": 0,
            "type": 0,
            "x": 0,
            "y": 0,
            "z": 0,
            "angle": 0,
            "result": False,
        }
        while True:
            data = self._taskcontroller._wait_state(SysState.P_State_CAMERA, 0.5)
            if data == None:
                continue
            _data["mode"] = data["mode"]
            _data["type"] = data["type"]
            #   _data['time_duration'] = data['time_duration']
            _data["x"] = int(data["x"] * 100)
            _data["y"] = int(data["y"] * 100)
            _data["z"] = int(data["z"] * 100)
            _data["angle"] = int(data["angle"] * 100)
            _data["result"] = data["result"] == 1
            time.sleep(0.5)
            return _data

    def single_fly_Qrcode_align(self, mode, time_duration, search_radius, qr_id):
        self._taskcontroller.create_task(
            UserTask.S_Fly_Qr_align,
            {
                "mode": mode,
                "time_duration": time_duration,
                "search_radius": search_radius,
                "qr_id": qr_id,
            },
        )
        _data = {
            "x": None,
            "y": None,
            "z": None,
            "qr_id": None,
            "result": False,
            "yaw": None,
        }
        while True:
            data = self._taskcontroller._wait_state(
                SysState.P_State_QRRecognite_Deal, 0.5
            )
            if data == None:
                continue
            if data["status"] <= 1:
                continue
            _data["result"] = data["status"] == 2 or data["status"] == 3
            _data["x"] = int(data["x_com"] * 100)
            _data["y"] = int(data["y_com"] * 100)
            _data["z"] = int(data["z_com"] * 100)
            _data["yaw"] = int(data["yaw_com"])
            _data["qr_id"] = data["qr_id"]
            time.sleep(0.5)
            return _data

    def single_fly_getColor(self, Mode):

        self._taskcontroller.create_task(UserTask.S_Fly_ColorRecog, {"Mode": Mode})
        data = self._taskcontroller._wait_state(SysState.P_State_ColorRecog, 1)
        if data == None:
            return None
        time.sleep(0.5)
        return data

    # def multi_fly_prepare(self,'_token': token,  plane_id_start,'_token': token,  plane_id_end):
    # ! MISSING: Line (376)
    # ! MISSING: Line (377)
    # ! MISSING: Line (378)
    # ! MISSING: Line (379)
    # ! MISSING: Line (380)
    # ! MISSING: Line (381)
    # ! MISSING: Line (382)
    # ! MISSING: Line (383)
    # ! MISSING: Line (384)
    # ! MISSING: Line (385)
    # ! MISSING: Line (386)
    # ! MISSING: Line (387)
    # ! MISSING: Line (388)
    # ! MISSING: Line (389)
    # ! MISSING: Line (390)
    # ! MISSING: Line (391)
    # ! MISSING: Line (392)
    #

    def plane_fly_arm(self, token=0):
        data = self._get_plane_data("heartbeat")

        if data != None and data.drone_status == 2:
            return self._taskcontroller.create_task(
                UserTask.S_Fly_unlock, {"_token": token, "plane_id": self._resolve_command_id()}
            )

        print("")

    def plane_fly_disarm(self, token=0):

        return self._taskcontroller.create_task(
            UserTask.S_Fly_lock, {"_token": token, "plane_id": self._resolve_command_id()}
        )

    def enable_led(self, token=0, blocking=True):
        """Enable LED - formation cmd 0x0C (12)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Enable_LED, {"_token": token, "plane_id": self._resolve_command_id(), "blocking": blocking}
        )

    def disable_led(self, token=0, blocking=True):
        """Disable LED - formation cmd 0x0D (13)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Disable_LED, {"_token": token, "plane_id": self._resolve_command_id(), "blocking": blocking}
        )

    def cancel_rgb(self, token=0, blocking=True):
        """Cancel RGB animation - formation cmd 0x1B (27)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Cancel_RGB, {"_token": token, "plane_id": self._resolve_command_id(), "blocking": blocking}
        )

    def vertical_circle(self, radius: float | int, token: int = 0, blocking: bool = True):
        """Vertical circle maneuver - formation cmd 0x27 (39)

        Args:
            radius: Circle radius in cm (positive=CCW, negative=CW)
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Vertical_Circle,
            {"_token": token, "plane_id": self._resolve_command_id(), "radius": radius, "blocking": blocking},
        )

    def set_avoidance(
        self,
        direction: int,
        barrier_mask: int = 0x3F,
        x: int = 0,
        y: int = 0,
        token: int = 0,
        blocking: bool = True,
    ):
        """Set avoidance with direction - formation cmd 0x2A (42)

        Args:
            direction: 0=forward, 1=back, 2=left, 3=right, 4/5=up/down
            barrier_mask: Bitmask for obstacle directions (default 0x3F = all)
            x: Forward/back distance in cm
            y: Left/right distance in cm
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Avoidance,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "direction": direction,
                "barrier_mask": barrier_mask,
                "x": x,
                "y": y,
                "blocking": blocking,
            },
        )

    def get_product_id(self, token=0):
        """Get product ID/autopilot version - formation cmd 0x2C (44)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Get_Product_ID, {"_token": token, "plane_id": self._resolve_command_id()}
        )

    def set_velocity(
        self,
        level: int,
        horizontal_vel: int = 0,
        token: int = 0,
        blocking: bool = True,
    ):
        """Set velocity level - formation cmd 0x30 (48)

        Args:
            level: Velocity in cm/s (0-300): 100=1.0m/s, 200=2.0m/s, 300=3.0m/s
            horizontal_vel: Optional horizontal velocity override in cm/s
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Velocity,
            {"_token": token, "plane_id": self._resolve_command_id(), "level": level, "horizontal_vel": horizontal_vel, "blocking": blocking},
        )

    def set_yawrate(self, level: int, token: int = 0, blocking: bool = True):
        """Set yaw rate level - formation cmd 0x31 (49)

        Args:
            level: Yaw rate level
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Yawrate,
            {"_token": token, "plane_id": self._resolve_command_id(), "level": level, "blocking": blocking},
        )

    def set_rgb_brightness(self, brightness: int, token: int = 0, blocking: bool = True):
        """Set RGB brightness - formation cmd 0x32 (50)

        Args:
            brightness: Brightness level
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_RGB_Brightness,
            {"_token": token, "plane_id": self._resolve_command_id(), "brightness": brightness, "blocking": blocking},
        )

    def enable_battery_failsafe(self, token=0):
        """Enable battery failsafe - formation cmd 0x35 (53)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Enable_Battery_FS, {"_token": token, "plane_id": self._resolve_command_id()}
        )

    def disable_battery_failsafe(self, token=0):
        """Disable battery failsafe - formation cmd 0x36 (54)"""
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Disable_Battery_FS, {"_token": token, "plane_id": self._resolve_command_id()}
        )

    def set_parameter(
        self,
        velocity: int = 0,
        yaw_rate: int = 0,
        brightness: int = 0,
        avoidance: bool = False,
        battery_failsafe: bool = False,
        fast_land: bool = False,
        token: int = 0,
        blocking: bool = True,
    ):
        """Set multiple parameters - formation cmd 0x37 (55)

        Args:
            velocity: Velocity level (0-3)
            yaw_rate: Yaw rate level
            brightness: RGB brightness
            avoidance: Enable obstacle avoidance
            battery_failsafe: Enable battery failsafe
            fast_land: Enable fast landing
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Parameter,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "velocity": velocity,
                "yaw_rate": yaw_rate,
                "brightness": brightness,
                "avoidance": avoidance,
                "battery_failsafe": battery_failsafe,
                "fast_land": fast_land,
                "blocking": blocking,
            },
        )

    def set_operate(self, status: int, token: int = 0):
        """Set formation operate status - formation cmd 0x3B (59)

        Args:
            status: Operate status value
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Operate,
            {"_token": token, "plane_id": self._resolve_command_id(), "status": status},
        )

    def set_land_speed(self, fast: bool = False, token: int = 0):
        """Set land speed - formation cmd 0x3C (60)

        Args:
            fast: True for fast landing, False for slow landing
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Land_Speed,
            {"_token": token, "plane_id": self._resolve_command_id(), "fast": fast},
        )

    def set_video_resolution(self, resolution: int, token: int = 0):
        """Set video resolution - plane cmd 0x16 (22)

        Controls RTP streaming and recording resolution.
        Lower resolution = less encoder CPU load = potentially better QR rate.

        Args:
            resolution: 0=HIGH (1080p), 1=MEDIUM (720p), 2=LOW (program/AI mode)
                       Can also use VideoResolution enum values.

        Firmware handler: HandleMsgSelectRecordResolution @ avmanager:0x0001df7c
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_Video_Resolution,
            {"_token": token, "plane_id": self._resolve_command_id(), "resolution": int(resolution)},
        )

    def set_wifi_mode(self, wifi_mode: int, channel_id: int = 0, token: int = 0):
        """Set WiFi mode - plane cmd 0x04 (4)

        Controls WiFi band, power, broadcast, and channel settings.

        Args:
            wifi_mode: WiFiMode enum value or int (0-9):

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
            channel_id: Channel ID for manual channel mode (wifi_mode=7)

        Firmware handler: HandleMsgWifiMode @ avmanager
        """
        return self._taskcontroller.create_task(
            UserTask.S_Fly_Set_WiFi_Mode,
            {
                "_token": token,
                "plane_id": self._resolve_command_id(),
                "wifi_mode": int(wifi_mode),
                "channel_id": channel_id,
            },
        )

    # =============================================Manual Control===================================================================#

    def send_manual_control(
        self,
        x: int = 0,
        y: int = 0,
        z: int = 0,
        r: int = 0,
        target: int = 0,
        buttons: int = 0,
    ) -> bool:
        """Send a single MANUAL_CONTROL frame via UDP.

        This provides joystick-style control with simultaneous position and yaw movement.
        Call this at ~20Hz (every 50ms) for smooth control.

        Args:
            x: Pitch (forward/back), -1000 to +1000. +1000 = full forward.
            y: Roll (left/right), -1000 to +1000. +1000 = full right.
            z: Throttle (up/down), -1000 to +1000. +1000 = full up.
            r: Yaw (rotation), -1000 to +1000. +1000 = full CCW.
            target: Target system ID (default 0).
            buttons: Button bitmask (default 0).

        Returns:
            True if message was sent successfully.
        """
        return self._taskcontroller.send_manual_control(x, y, z, r, target, buttons)

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
        """Fly with manual control inputs for a specified duration.

        Sends continuous MANUAL_CONTROL messages at the specified rate.
        Also sends APP_HEARTBEAT with user_mode=1 (Aerial mode) every second
        as required by the drone firmware for manual control.

        Args:
            duration_sec: How long to fly with these inputs (seconds).
            forward: Forward/back input, -1.0 to +1.0. +1.0 = full forward.
            right: Left/right input, -1.0 to +1.0. +1.0 = full right.
            up: Up/down input, -1.0 to +1.0. +1.0 = full up.
            rotate: Rotation input, -1.0 to +1.0. +1.0 = full CCW.
            rate_hz: Control loop rate (default 20 Hz = 50ms interval).
            on_frame: Optional callback(x, y, z, r, frame_index, success) called after each frame.

        Returns:
            True if all messages were sent successfully.

        Example:
        ```python
        # Move forward while rotating left for 2 seconds
        server.manual_fly(2.0, forward=0.5, rotate=0.3)
        ```
        """
        # Convert -1.0 to +1.0 range to -1000 to +1000
        x = int(forward * 1000)
        y = int(right * 1000)
        z = int(up * 1000)
        r = int(rotate * 1000)

        interval = 1.0 / rate_hz
        iterations = int(duration_sec * rate_hz)

        # Set app mode to Aerial (1) for manual control - this affects background heartbeat
        old_mode = self._taskcontroller.get_app_mode()
        self._taskcontroller.set_app_mode(1)  # Aerial mode

        success = True
        for i in range(iterations):
            frame_success = self._taskcontroller.send_manual_control(x, y, z, r)
            if not frame_success:
                success = False
            if on_frame:
                on_frame(x, y, z, r, i, frame_success)
            time.sleep(interval)

        # Send zero inputs to stop
        stop_success = self._taskcontroller.send_manual_control(0, 0, 0, 0)
        if on_frame:
            on_frame(0, 0, 0, 0, iterations, stop_success)

        # Restore previous app mode
        self._taskcontroller.set_app_mode(old_mode)

        return success

    def stop_manual_control(self) -> bool:
        """Send zero inputs to stop manual movement.

        Returns:
            True if message was sent successfully.
        """
        return self._taskcontroller.send_manual_control(0, 0, 0, 0)

    def send_app_heartbeat(self, user_mode: int = 1) -> bool:
        """Send APP_HEARTBEAT message to set the app control mode.

        The drone requires periodic heartbeats to accept certain control modes.
        Call this before and during manual_control_frame() calls.

        user_mode values:
            0 = Other
            1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
            2 = Program (autonomous flight mode)
            3 = Battle
            4 = Formation

        Args:
            user_mode: App mode (0-4). Default 1 for manual flight.

        Returns:
            True if message was sent successfully.
        """
        return self._taskcontroller.send_app_heartbeat(user_mode)

    def set_app_mode(self, mode: int) -> None:
        """Set the app mode for background heartbeat messages.

        This changes the user_mode sent in periodic APP_HEARTBEAT messages.
        The background heartbeat thread sends this mode every second.

        Mode values:
            0 = Other
            1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
            2 = Program (autonomous flight mode) - default
            3 = Battle
            4 = Formation

        Args:
            mode: App mode (0-4).
        """
        self._taskcontroller.set_app_mode(mode)

    def get_app_mode(self) -> int:
        """Get the current app mode for background heartbeat messages."""
        return self._taskcontroller.get_app_mode()
