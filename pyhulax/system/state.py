import time
import enum

SysState = enum.Enum(
    "SysState",
    (
        "U_State_Get",
        "U_State_GetHeartbeat",
        "M_Dance_GetFormationRequest",
        "P_State_GetHeartbeat",  
        "P_Ack_GetFormation",  
        "P_Ack_GetAuxSetup",  
        "P_Sys_Response",  
        "P_State_Photoresponse",  
        "P_State_Plight_Data",  
        "p_State_BROADCAST_PLANE_STATUS",  
        "P_State_CAMERA",  
        "P_State_QRRecognite_Deal",  
        "P_State_ColorRecog",  
        "P_State_WALKING",  
        "P_State_MSG_ID_PLANE_ACK",  
    ),  
)  


class State:
    __slots__ = ("_state", "_data", "_time")

    def __init__(self, state, data=None):
        if state not in SysState:
            raise ValueError
        self._state = state
        self._data = data
        self._time = time.time()

    def set_state(self, state):
        if state not in SysState:
            raise ValueError
        self._state = state

    def set_data(self, data):
        self._data = data

    def get_state(self):
        return self._state

    def get_data(self):
        return self._data

    def updata_time(self):
        self._time = time.time()

    def get_time(self):
        return self._time

    def get_str_time(self):
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self._time))


if __name__ == "__main__":
    us = State(SysState.U_State_Get, 88)
    print(us.get_state())
    print(us.get_data())
    print(us.get_time())
    print(us.get_str_time())

    us.set_state(SysState.U_State_GetHeartbeat)
    us.set_data(20)
    us.updata_time()
    print(us.get_state())
    print(us.get_data())
    print(us.get_time())
    print(us.get_str_time())
