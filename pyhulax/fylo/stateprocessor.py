import enum

from . import mavlink

from ..system import state
from ..system.datacenter import *
import os
from . import config
from pyhulax.config import DroneConfig, get_config

_cfg = get_config()

# print(config.drone_id,55555555555555)
MsgType = enum.Enum("MsgType", ("UwbMsg", "MavlinkMsg"))

_datacenter = DataCenter()


def refresh_runtime_config(runtime_config: DroneConfig) -> None:
    """Refresh battery thresholds after runtime config changes."""
    Plane_REPORT_FLIGHT_DATA.BATTERY_WARNING_THRESHOLD = (
        runtime_config.battery.warning_threshold
    )
    Plane_REPORT_FLIGHT_DATA.BATTERY_CRITICAL_THRESHOLD = (
        runtime_config.battery.critical_threshold
    )
    Plane_REPORT_FLIGHT_DATA._battery_warning_emitted = False


class StateProcessor:
    mavlink_msg_id = None
    uwb_msg_id = None

    def __init__(self, msg, msg_type):

        if msg_type == MsgType.UwbMsg:
            self.uwb_msg_id = msg.get_msg_id()
        elif msg_type == MsgType.MavlinkMsg:
            self.mavlink_msg_id = msg.get_msg_id()
        self._msg = msg

    def get_state(self) -> "state.State | None":
        pass


class FormationRequestInfoSP(StateProcessor):
    mavlink_msg_id = 173

    def get_state(self):
        _state = state.State(
            state.SysState.M_Dance_GetFormationRequest,
            {"target_info": self._msg.target_info, "dance_id": self._msg.dance_id},
        )
        # print('get request', 'target_info', self._msg.target_info, 'dance_id', self._msg.dance_id)
        return _state

    def get_pack_info(self):
        return {"state_id": state.SysState.M_Dance_GetFormationRequest}


class FirmwareHeadsSP(StateProcessor):
    mavlink_msg_id = 184

    def get_state(self):
        return None


class HeartbeatSP(StateProcessor):
    mavlink_msg_id = 207

    def get_state(self):
        _datacenter.set_data("Plane", "heartbeat", self._msg)
        # Create proper State object for event system
        _state = state.State(
            state.SysState.P_State_GetHeartbeat,
            {
                "drone_status": self._msg.drone_status,
                "block_status": self._msg.block_status,
                "battery_volumn": self._msg.battery_volumn,
                "drone_id": getattr(self._msg, 'drone_id', None),
            },
        )
        return _state


#! MISSING: Line (62)        _state = state.State(state.SysState.P_State_GetHeartbeat,





















class Plane_REPORT_FLIGHT_DATA(StateProcessor):
    mavlink_msg_id = 206

    # Low battery warning threshold - emit warning instead of killing process
    BATTERY_WARNING_THRESHOLD = _cfg.battery.warning_threshold
    BATTERY_CRITICAL_THRESHOLD = _cfg.battery.critical_threshold
    _battery_warning_emitted = False

    def get_state(self):
        _datacenter.set_data("Plane", "flight_data", self._msg)

        # Emit battery warnings instead of killing process
        battery = self._msg.battery_volumn
        if battery < self.BATTERY_CRITICAL_THRESHOLD:
            print(f"[CRITICAL] Battery at {battery}% - landing recommended!")
            # Return state to notify listeners instead of os._exit()
            return state.State(
                state.SysState.P_State_Plight_Data,
                {"battery_critical": True, "battery_percent": battery},
            )
        elif battery < self.BATTERY_WARNING_THRESHOLD and not Plane_REPORT_FLIGHT_DATA._battery_warning_emitted:
            print(f"[WARNING] Battery at {battery}%")
            Plane_REPORT_FLIGHT_DATA._battery_warning_emitted = True

        return None
        #! MISSING: Line (92)        _state = state.State(state.SysState.P_Sys_Response,{'roll':self._msg.roll,'pitch':self._msg.pitch,
        #! MISSING: Line (93) 'yaw':self._msg.yaw,'accx':self._msg.accx,
        
        
        
        
        # 'distance':self._msg.distance})
        return _state


class PhotoresponseSP(StateProcessor):
    mavlink_msg_id = 230

    def get_state(self):
        # self._datacenter.set_data('Plane', 'EXTEND', self._msg)

        _state = state.State(
            state.SysState.P_State_Photoresponse,
            {
                "extend": self._msg.extend,
                "token": self._msg.token,
                "result": self._msg.result,
                "type": self._msg.type,
                "cmd": self._msg.cmd,
            },
        )
        return _state


class FormationAckSP(StateProcessor):
    mavlink_msg_id = 209

    def get_state(self):

        _state = state.State(
            state.SysState.P_Ack_GetFormation,
            {
                "plane_id": self._msg.id,
                "token": self._msg.token,
                "result": self._msg.result,
                "type": self._msg.type,
                "cmd": self._msg.cmd,
            },
        )

        return _state


class PSysResponseSP(StateProcessor):
    mavlink_msg_id = 213

    def get_state(self):

        _state = state.State(
            state.SysState.P_Sys_Response,
            {"plane_id": self._msg.plane_id, "target_info": self._msg.target_info},
        )

        return _state


class BROADCAST_PLANE_STATUS(StateProcessor):
    mavlink_msg_id = 232

    def get_state(self):
        # print('get auxsetup ack')

        # Always record what the drone advertises so connect() can surface it
        # for diagnostics even when bind_client was already detected locally.
        config.drone_reported_bind_client = getattr(self._msg, "bind_client", None)

        if config.bind_client == None or config.bind_client == 255:
            config.bind_client = self._msg.bind_client
            print(self._msg)
        return None
        _state = state.State(
            state.SysState.p_State_BROADCAST_PLANE_STATUS,
            {
                "plane_id": self._msg.id,
                "bind_client": self._msg.bind_client,
                "sn": self._msg.sn,
                "ip": self._msg.ip,
                "wifi_mode": self._msg.wifi_mode,
            },
        )
        
        return _state


class PLANE_STATUS(StateProcessor):
    mavlink_msg_id = 231

    def get_state(self):
        if config.drone_id == None:
            config.drone_id = self._msg.plane_id
            print(self._msg)

        return None
        #! MISSING: Line (154)        _state = state.State(state.SysState.P_State_Plight_Data, {'capacity': self._msg.capacity, 'remaining': self._msg.remaining,
        #! MISSING: Line (155)                                                                  'occupied': self._msg.occupied,'plane_id': self._msg.plane_id,
        
        
        
        return _state


class PLANE_CAMERA(StateProcessor):
    mavlink_msg_id = 236

    def get_state(self):

        _state = state.State(
            state.SysState.P_State_CAMERA,
            {
                "mode": self._msg.mode,
                "type": self._msg.type,
                "result": self._msg.result,
                "x": self._msg.x,
                "y": self._msg.y,
                "z": self._msg.z,
                "angle": self._msg.angle,
                "time_duration": self._msg.time_duration,
            },
        )

        return _state


class PLANE_QRRecognite_Deal(StateProcessor):
    mavlink_msg_id = 234

    def get_state(self):

        _state = state.State(
            state.SysState.P_State_QRRecognite_Deal,
            {
                "time_duration": self._msg.time_duration,
                "yaw_com": self._msg.yaw_com,
                "search_radius": self._msg.search_radius,
                "run_rate": self._msg.run_rate,
                "qr_id": self._msg.qr_id,
                "x_com": self._msg.x_com,
                "y_com": self._msg.y_com,
                "z_com": self._msg.z_com,
                "mode": self._msg.mode,
                "qr_background_grayscale": self._msg.qr_background_grayscale,
                "status": self._msg.status,
            },
        )

        return _state


class PLANE_ColorRecog(StateProcessor):
    mavlink_msg_id = 235

    def get_state(self):

        _state = state.State(
            state.SysState.P_State_ColorRecog,
            {
                "r": self._msg.r,
                "g": self._msg.g,
                "b": self._msg.b,
                "state": self._msg.state == 1,
            },
        )
        return _state


class mavlink_statustext_t(StateProcessor):
    mavlink_msg_id = 253

    def get_state(self):
        # Log status text instead of killing process
        text = getattr(self._msg, 'text', 'Unknown status')
        severity = getattr(self._msg, 'severity', 0)
        print(f"[STATUSTEXT] Severity {severity}: {text}")
        return None


class mavlink_WALKING(StateProcessor):
    mavlink_msg_id = 219

    def get_state(self):
        _state = state.State(
            state.SysState.P_State_WALKING, {"result": self._msg.result}
        )
        return _state


class mavlink_MSG_ID_PLANE_ACK(StateProcessor):
    mavlink_msg_id = 228

    def get_state(self):

        _state = state.State(
            state.SysState.P_State_MSG_ID_PLANE_ACK,
            {"result": self._msg.result, "cmd": self._msg.cmd, "type": self._msg.type},
        )
        return _state


# ============================================================================
# Additional State Processors for All Documented Message IDs
# ============================================================================

class SYSTEM_TIME_SP(StateProcessor):
    """SYSTEM_TIME (msg 2) - Time sync from app"""
    mavlink_msg_id = 2

    def get_state(self):
        _datacenter.set_data("Plane", "system_time", self._msg)
        return None


class MANUAL_CONTROL_SP(StateProcessor):
    """MANUAL_CONTROL (msg 69) - Joystick input"""
    mavlink_msg_id = 69

    def get_state(self):
        _datacenter.set_data("Plane", "manual_control", self._msg)
        return None


class MANUAL_CONTROL2_SP(StateProcessor):
    """MANUAL_CONTROL2 (msg 71) - Follow mode joystick"""
    mavlink_msg_id = 71

    def get_state(self):
        _datacenter.set_data("Plane", "manual_control2", self._msg)
        return None


class LOCAL_POSITION_SP(StateProcessor):
    """LOCAL_POSITION (msg 72) - HIGH-FREQUENCY 500Hz position stream"""
    mavlink_msg_id = 72

    def get_state(self):
        # Store high-frequency position data
        _datacenter.set_data("Plane", "local_position", self._msg)
        return None


class OPTITRACK_SP(StateProcessor):
    """OPTITRACK (msg 101) - External motion capture data"""
    mavlink_msg_id = 101

    def get_state(self):
        _datacenter.set_data("Plane", "optitrack", self._msg)
        return None


class AUTOPILOT_VERSION_SP(StateProcessor):
    """AUTOPILOT_VERSION (msg 148) - Product info"""
    mavlink_msg_id = 148

    def get_state(self):
        _datacenter.set_data("Plane", "autopilot_version", self._msg)
        return None


class POSITION_CONTROL_SETPOINT_SP(StateProcessor):
    """POSITION_CONTROL_SETPOINT (msg 170) - Target position control"""
    mavlink_msg_id = 170

    def get_state(self):
        _datacenter.set_data("Plane", "position_setpoint", self._msg)
        return None


class APP_HEARTBEAT_SP(StateProcessor):
    """APP_HEARTBEAT (msg 204) - Keep-alive from app"""
    mavlink_msg_id = 204

    def get_state(self):
        _datacenter.set_data("Plane", "app_heartbeat", self._msg)
        return None


class FORMATION_CMD_SP(StateProcessor):
    """FORMATION_CMD (msg 208) - Flight commands sent from app"""
    mavlink_msg_id = 208

    def get_state(self):
        _datacenter.set_data("Plane", "formation_cmd", self._msg)
        return None


class FILE_RESPONSE_INFO_SP(StateProcessor):
    """FILE_RESPONSE_INFO (msg 213) - File transfer response"""
    mavlink_msg_id = 213

    def get_state(self):
        _datacenter.set_data("Plane", "file_response", self._msg)
        return None


class WIFI_SETS_SP(StateProcessor):
    """WIFI_SETS (msg 214) - WiFi configuration"""
    mavlink_msg_id = 214

    def get_state(self):
        _datacenter.set_data("Plane", "wifi_sets", self._msg)
        return None


class COLOR_TRACK_TARGET_SP(StateProcessor):
    """COLOR_TRACK_TARGET (msg 215) - Color tracking setup"""
    mavlink_msg_id = 215

    def get_state(self):
        _datacenter.set_data("Plane", "color_track_target", self._msg)
        return None


class COLOR_TRACK_TARGET_RET_SP(StateProcessor):
    """COLOR_TRACK_TARGET_RET (msg 216) - Color tracking result"""
    mavlink_msg_id = 216

    def get_state(self):
        _datacenter.set_data("Plane", "color_track_result", self._msg)
        return None


class FILE_HEADS_SP(StateProcessor):
    """FILE_HEADS (msg 227) - File transfer header"""
    mavlink_msg_id = 227

    def get_state(self):
        _datacenter.set_data("Plane", "file_heads", self._msg)
        return None


class PLANE_COMMAND_SP(StateProcessor):
    """PLANE_COMMAND (msg 229) - Camera/laser/magnet commands to Linux"""
    mavlink_msg_id = 229

    def get_state(self):
        _datacenter.set_data("Plane", "plane_command", self._msg)
        return None


class LONGITUDE_LATITUDE_SP(StateProcessor):
    """LONGITUDE_LATITUDE (msg 237) - GPS coordinates"""
    mavlink_msg_id = 237

    def get_state(self):
        _datacenter.set_data("Plane", "gps_coords", self._msg)
        return None


class CONSOLE_SP(StateProcessor):
    """CONSOLE (msg 252) - Debug console commands"""
    mavlink_msg_id = 252

    def get_state(self):
        _datacenter.set_data("Plane", "console", self._msg)
        return None


# ============================================================================
# Complete State Processor List (All Known Message IDs)
# ============================================================================

mavlink_state_processor_list = [
    # Original processors
    Plane_REPORT_FLIGHT_DATA,
    HeartbeatSP,
    FormationAckSP,
    PLANE_STATUS,
    BROADCAST_PLANE_STATUS,
    PLANE_CAMERA,
    PLANE_QRRecognite_Deal,
    PLANE_ColorRecog,
    mavlink_statustext_t,
    mavlink_WALKING,
    mavlink_MSG_ID_PLANE_ACK,
    PhotoresponseSP,
    # NEW: Previously missing processors
    SYSTEM_TIME_SP,              # 2
    MANUAL_CONTROL_SP,           # 69
    MANUAL_CONTROL2_SP,          # 71
    LOCAL_POSITION_SP,           # 72 - CRITICAL: 500Hz stream!
    OPTITRACK_SP,                # 101
    AUTOPILOT_VERSION_SP,        # 148
    POSITION_CONTROL_SETPOINT_SP,# 170
    APP_HEARTBEAT_SP,            # 204
    FORMATION_CMD_SP,            # 208
    FILE_RESPONSE_INFO_SP,       # 213
    WIFI_SETS_SP,                # 214
    COLOR_TRACK_TARGET_SP,       # 215
    COLOR_TRACK_TARGET_RET_SP,   # 216
    FILE_HEADS_SP,               # 227
    PLANE_COMMAND_SP,            # 229
    LONGITUDE_LATITUDE_SP,       # 237
    CONSOLE_SP,                  # 252
]


class UwbHeartBeatSP(StateProcessor):
    uwb_msg_id = 1

    def get_state(self):
        # print('uwb get heartbeat')
        return None


class UwbStateSP(StateProcessor):
    uwb_msg_id = 2

    def get_state(self):
        _state = state.State(
            state.SysState.U_State_Get,
            {
                "uwbstate": self._msg,
                "translatemode": self._msg.translate_mode,
                "X0": self._msg.a0_x,
                "X1": self._msg.a1_x,
                "X2": self._msg.a2_x,
                "X3": self._msg.a3_x,
                "Y0": self._msg.a0_y,
                "Y1": self._msg.a1_y,
                "Y2": self._msg.a2_y,
                "Y3": self._msg.a3_y,
                "Z0": self._msg.a0_z,
                "Z1": self._msg.a1_z,
                "Z2": self._msg.a2_z,
                "Z3": self._msg.a3_z,
            },
        )

        
        
        
        
        
        # print(self._msg.translate_mode)
        # print(_state)
        return _state


class UwbLocationResponse(StateProcessor):
    uwb_msg_id = 3

    def get_state(self):
        print("get location response")
        return None


uwb_state_process_list = [UwbHeartBeatSP, UwbStateSP, UwbLocationResponse]


class StateProcessorFactory:
    @staticmethod
    def get_state_processor(msg):
        if isinstance(msg, mavlink.MAVLink_message):
            for cl in mavlink_state_processor_list:
                if cl.mavlink_msg_id == msg.get_msg_id():
                    return cl(msg, MsgType.MavlinkMsg)
        return None


if __name__ == "__main__":
    mav = mavlink.MAVLink(None, src_system=1, src_component=1)
    msg = mav.app_heartbeat_encode(0)

    sp = StateProcessorFactory.get_state_processor(msg)
    print(type(sp))
