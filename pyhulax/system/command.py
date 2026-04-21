import enum

SysCommand = enum.Enum(
    "SysCommand",
    (
        "S_Fly_Takeoff",
        "S_Fly_Touchdown",
        "S_Fly_Forward",
        "S_Fly_Back",
        "S_Fly_Left",
        "S_Fly_Right",
        "S_Fly_Up",
        "S_Fly_Down",
        "S_Fly_TurnLeft",
        "S_Fly_unlock",
        "S_Fly_lock",
        "S_Fly_RadiusAround",
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


class Command:

    __slots__ = ("_command", "_data")

    def __init__(self, cmd, data=None):
        if cmd not in SysCommand:
            raise ValueError
        self._command = cmd
        self._data = data

    def set_command(self, cmd):
        if cmd not in SysCommand:
            raise ValueError
        self._command = cmd

    def set_data(self, data):
        self._data = data

    def get_command(self):
        return self._command

    def get_data(self):
        return self._data


if __name__ == "__main__":
    cmd = Command(SysCommand.S_Fly_Takeoff, "1")

    print(cmd.get_command())
    print(cmd.get_data())

    cmd.set_command(SysCommand.S_Fly_Touchdown)
    cmd.set_data("2")

    print(cmd.get_command())
    print(cmd.get_data())
