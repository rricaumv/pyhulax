from __future__ import annotations

from enum import Enum
from threading import Lock
from typing import Optional, TYPE_CHECKING

from ..core.models import Obstacles, FlightData

Device = Enum("Device", ("Uwb", "Plane", "Sys"))

PlaneType = Enum(
    "PlaneType",
    (
        "Height",
        "Yaw",
        "PlaneX",
        "PlaneY",
        "PlaneZ",
        "TimeToken",
        "AuxToken",
        "Battery",
        "RangeSafe",
        "TakeoffAllow",
        "LocationSensor",
        "DanceX",
        "DanceY",
        "DanceZ",
        "RealDanceMD5",
        "DanceName",
        "Time",
        "Rgb_status",
        "Socket",
        "Address",
    ),
)

UwbType = Enum(
    "UwbType",
    (
        "DemarcateState",
        "Station0_X",
        "Station0_Y",
        "Station0_Z",
        "Station1_X",
        "Station1_Y",
        "Station1_Z",
        "Station2_X",
        "Station2_Y",
        "Station2_Z",
        "Station3_X",
        "Station3_Y",
        "Station3_Z",
    ),
)

SysType = Enum(
    "SysType", ("Delta_T", "TimeToken", "AuxToken", "DanceMD5", "AuxSetupYaw")
)


class DataCenter:
    __instance = None

    def __new__(cls):
        if cls.__instance is None:
            cls.__instance = object.__new__(cls)
            cls.__instance._flag = 1
            return cls.__instance
        else:
            return cls.__instance

    def __init__(self):
        if self._flag == 1:
            self._flag = 0
            self._data = {}
            self._lock = Lock()  #

    def empty_datacenter(self):
        self._data = {}

    def set_data(self, device, datatype, value, id=0):
        # if device not in Device or (datatype not in PlaneType and datatype not in UwbType and datatype not in SysType) or id < 0:
        #   return None
        with self._lock:  #
            if device not in self._data:
                self._data[device] = {}

            if id not in self._data[device]:
                self._data[device][id] = {}

            self._data[device][id][datatype] = value

    def get_data(self, device, datatype, id=0):
        # if device not in Device or (datatype not in PlaneType and datatype not in UwbType and datatype not in SysType) or id < 0:
        #   return None
        with self._lock:
            if device not in self._data:
                return None
            elif id not in self._data[device]:
                return None
            elif datatype not in self._data[device][id]:
                return None
            else:
                data = self._data[device][id][datatype]
                return data
            # return self._data[device][id][datatype]

    def device_exist(self, device):
        if device not in self._data:
            return False
        if device not in self._data:
            return False
        return True

    def id_exist(self, device, id):
        if device not in self._data:
            return False
        if id not in self._data[device]:
            return False
        return True

    def datatype_exist(self, device, id, datatype):
        if device not in self._data:
            return False
        if id not in self._data[device]:
            return False
        if datatype not in self._data[device][id]:
            return False
        return True

    def get_device_list(self):
        _list = [device for device in self._data]
        if len(_list) == 0:
            return None
        else:
            return _list

    def get_id_list(self, device):
        if device in self._data:

            _list = [id for id in self._data[device]]

            _list.sort()
            _list_pop = _list
            if len(_list_pop):
                while _list_pop[-1] > 254:
                    # print('plane_list: ', _list)
                    _list_pop.pop()
                    if _list_pop == []:
                        break
            if len(_list_pop):
                while _list_pop[0] < 0:
                    _list_pop.pop(0)
                    if _list_pop == []:
                        break
            if len(_list_pop) == 0:
                return None
            else:
                return _list_pop

    def get_datatype_list(self, device, id):
        if device in self._data:
            if id in self._data[device]:
                _list = [datatype for datatype in self._date[device][id]]
                if len(_list) == 0:
                    return None
                else:
                    return _list

    def del_device(self, device):
        if device in self._data:
            self._data.pop(device)

    def del_id(self, device, id):
        if device in self._data:
            if id in self._data[device]:
                self._data[device].pop(id)

    def del_datatype(self, device, id, datatype):
        if device in self._data:
            if id in self._data[device]:
                if datatype in self._data[device][id]:
                    self._data[device][id].pop(datatype)

    def show_data(self):
        for device in self._data:
            for id in self._data[device]:
                for type in self._data[device][id]:
                    print(
                        "device:",
                        device,
                        " id:",
                        id,
                        " datatype:",
                        type,
                        " value",
                        self._data[device][id][type],
                    )

    # ==================== Typed Accessors ====================
    # These methods return properly typed Pydantic models instead of raw data

    def get_flight_data(self, drone_id: int = 0) -> Optional[FlightData]:
        """
        Get typed flight telemetry data.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            FlightData model or None if not available
        """
        from ..core.models import FlightData

        raw = self.get_data("Plane", "flight_data", drone_id)
        if raw is None:
            return None
        return FlightData.from_mavlink(raw)

    def get_battery_percent(self, drone_id: int = 0) -> Optional[int]:
        """
        Get battery percentage.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            Battery percentage (0-100) or None if not available
        """
        raw = self.get_data("Plane", "flight_data", drone_id)
        if raw is None:
            return None
        return int(raw.battery_volumn)

    def get_obstacles(self, drone_id: int = 0) -> Obstacles:
        """
        Get obstacle detection state.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            Obstacles model (empty if no data)
        """
        from ..core.models import Obstacles

        raw = self.get_data("Plane", "flight_data", drone_id)
        if raw is None:
            return Obstacles()
        return Obstacles.from_bitmask(raw.barrier)

    def get_drone_status(self, drone_id: int = 0) -> Optional[int]:
        """
        Get drone status from heartbeat.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            Drone status code or None if not available
        """
        raw = self.get_data("Plane", "heartbeat", drone_id)
        if raw is None:
            return None
        return int(raw.drone_status)

    def is_drone_ready(self, drone_id: int = 0) -> bool:
        """
        Check if drone is in ready state.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            True if drone is ready (status == 2)
        """
        status = self.get_drone_status(drone_id)
        return status == 2 if status is not None else False

    def has_telemetry(self, drone_id: int = 0) -> bool:
        """
        Check if telemetry data is available.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            True if flight_data exists
        """
        return self.get_data("Plane", "flight_data", drone_id) is not None

    def has_heartbeat(self, drone_id: int = 0) -> bool:
        """
        Check if heartbeat data is available.

        Args:
            drone_id: Drone ID (default 0)

        Returns:
            True if heartbeat exists
        """
        return self.get_data("Plane", "heartbeat", drone_id) is not None


if __name__ == "__main__":

    dc = DataCenter()

    # dc.set_data(Device.Plane, PlaneType.Height, 100, 1)
    
    
    
    
    
    
    
    # dt = dc.get_data(Device.Uwb, UwbType.DemarcateState)

    dc.set_data(Device.Plane, PlaneType.Yaw, "1", 1)
    dc.set_data(Device.Plane, PlaneType.Lon, "2", 2)
    print(dc.get_id_list(Device.Plane))
    # dt = dc.get_data(Device.Plane, PlaneType.Height, 1)
    # print(dt)
