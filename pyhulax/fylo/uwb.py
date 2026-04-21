UWB_MSG_ID_HEARTBEAT = 1
UWB_MSG_ID_STATE = 2
UWB_MSG_ID_LOCATION_RESPONSE = 3
UWB_MSG_ID_FILE = 4
UWB_MSG_ID_SWITCH_MODE = 5
UWB_MSG_ID_GET_STATE = 6
UWB_MSG_ID_DEMARCATE = 7


class HeadError(Exception):
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class UwbError(Exception):
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class UwbMsg:
    msg_id = None
    msg_len = None
    msg_header = None
    msg_function_mark = None

    def decode(self, buf):
        return None

    def pack(self):
        return None

    def get_msg_id(self):
        return self.msg_id


class UwbHeartbeat(UwbMsg):
    msg_id = UWB_MSG_ID_HEARTBEAT
    msg_len = 896
    msg_header = 0x55
    msg_function_mark = 0x00
    msg_sumcheck = 0xEE

    def decode(self, buf):
        if (
                buf[0] != UwbHeartbeat.msg_header
                or buf[1] != UwbHeartbeat.msg_function_mark
        ):
            raise HeadError("UwbHeartbeat Decode Head error")
        if len(buf) < UwbHeartbeat.msg_len:
            raise UwbError("UwbHeartbeat Decode len error")
        if buf[UwbHeartbeat.msg_len - 1] != UwbHeartbeat.msg_sumcheck:
            raise UwbError("UwbHeartbeat Decode sumcheck error")
        return UwbHeartbeat()


class UwbState(UwbMsg):
    msg_id = UWB_MSG_ID_STATE
    msg_len = 128
    msg_header = 0x54
    msg_function_mark = 0x00

    @staticmethod
    def get_coordinate(buf):
        num = 0

        if buf[2] & 0x8 == 0:
            num += buf[0] << 1 * 8
            num += buf[1] << 2 * 8
            num += buf[2] << 3 * 8
        else:
            num += (0xFF - buf[0] + 1) << 1 * 8
            num += (0xFF - buf[1]) << 2 * 8
            num += (0xFF - buf[2]) << 3 * 8
            num = -num
        return num / 256 / 1000

    def decode(self, buf):
        if buf[0] != UwbState.msg_header or buf[1] != UwbState.msg_function_mark:
            raise HeadError("UwbState Decode Head error")
        if len(buf) < UwbState.msg_len:
            raise UwbError("UwbState Decode len error")

        sum_val = 0
        for i in range(UwbState.msg_len - 1):
            sum_val += buf[i]
        if sum_val & 0xFF != buf[UwbState.msg_len - 1]:
            raise UwbError("UwbState Decode sumcheck error")

        us = UwbState()
        us.a0_x = 0
        us.a0_y = 0
        us.a0_z = 0
        us.a1_x = 0
        us.a1_y = 0
        us.a1_z = 0
        us.a2_x = 0
        us.a2_y = 0
        us.a2_z = 0
        us.a3_x = 0
        us.a3_y = 0
        us.a3_z = 0
        if buf[17] == 0x03 and buf[18] == 0x04:
            us.translate_mode = "DataTran"
        elif buf[17] == 0x00 and buf[18] == 0xFF:
            us.translate_mode = "Location"
        elif buf[17] & 0x0F == 0x08:
            us.translate_mode = "Demarcate"
        else:
            us.translate_mode = "Normal"
            us.a0_x = UwbState.get_coordinate()
            us.a0_y = UwbState.get_coordinate()
            us.a0_z = UwbState.get_coordinate()
            us.a1_x = UwbState.get_coordinate()
            us.a1_y = UwbState.get_coordinate()
            us.a1_z = UwbState.get_coordinate()
            us.a2_x = UwbState.get_coordinate()
            us.a2_y = UwbState.get_coordinate()
            us.a2_z = UwbState.get_coordinate()
            us.a3_x = UwbState.get_coordinate()
            us.a3_y = UwbState.get_coordinate()
            us.a3_z = UwbState.get_coordinate()

        us.buf = buf
        # print(buf)
        return us

    def __init__(self):
        self.translate_mode = None
        self.buf = None


class UwbLocationResponse(UwbMsg):
    msg_id = UWB_MSG_ID_LOCATION_RESPONSE
    msg_len = 28
    msg_header = 0x55
    msg_function_mark = 0x03

    def decode(self, buf):
        if (
                buf[0] != UwbLocationResponse.msg_header
                or buf[1] != UwbLocationResponse.msg_function_mark
        ):
            raise HeadError("UwbLocationResponse Decode Head error")
        if len(buf) < UwbLocationResponse.msg_len:
            raise UwbError("UwbLocationResponse Decode len error")

        sum_val = 0
        for i in range(UwbLocationResponse.msg_len - 1):
            sum_val += buf[i]
        if sum_val & 0xFF != buf[UwbLocationResponse.msg_len - 1]:
            raise UwbError("UwbLocationResponse Decode sumcheck error")

        ulr = UwbLocationResponse()
        return ulr

    def __init__(self):
        pass


class UwbFile(UwbMsg):
    msg_id = UWB_MSG_ID_FILE

    def __init__(self, buf, target_type, target_no):
        if target_type != 0x00 and target_type != 0x01 and target_type != 0x02:
            raise UwbError("UwbFile target type error")
        if (not isinstance(target_no, int)) or target_no < 0 or target_no > 254:
            raise UwbError("UwbFile target no error")

        self._buf = buf
        self._buf_len = len(buf)
        self._len_buf = [self._buf_len & 0xFF, self._buf_len >> 8]
        self._target_type = target_type
        self._target_no = target_no

    def pack(self):
        buf = [
                  0x54,
                  0xF1,
                  0xFF,
                  0xFF,
                  0xFF,
                  0xFF,
                  self._target_type,
                  self._target_no,
              ] + self._len_buf
        buf += self._buf
        checknum = 0
        for i in range(len(buf)):
            checknum += buf[i]
        checknum = checknum & 0xFF
        buf += [checknum]
        return buf


class UwbSwitchMode(UwbMsg):
    msg_id = UWB_MSG_ID_SWITCH_MODE

    location_mode_list1 = [0x00, 0x00, 0x02, 0x00, 0x00]
    location_mode_list2 = [0xCB, 0x00, 0xCD, 0x00, 0x00]

    def __init__(self, translate_mode, location_mode=0):
        if translate_mode != "DataTran" and translate_mode != "Location":
            raise UwbError("UwbSwitchMode translate mode error")
        if (
                (not isinstance(location_mode, int))
                or location_mode > 4
                or location_mode < 0
        ):
            raise UwbError("UwbSwitchMode location mode error")

        self._translate_mode = translate_mode
        self._location_mode = location_mode

    def pack(self):
        buf = []
        for i in range(0, 128):
            buf += [0xFF]

        if self._translate_mode == "DataTran":
            buf[0] = 0x54
            buf[1] = 0xF0
            buf[2] = 0x02
            buf[17] = 0x03
            buf[127] = 0xCE
            return buf
        elif self._translate_mode == "Location":
            buf[0] = 0x54
            buf[1] = 0xF0
            buf[2] = 0x02
            buf[17] = UwbSwitchMode.location_mode_list1[self._location_mode]
            buf[127] = UwbSwitchMode.location_mode_list2[self._location_mode]
            return buf


class UwbGetState(UwbMsg):
    msg_id = UWB_MSG_ID_GET_STATE

    def __init__(self):
        pass

    def pack(self):
        buf = []
        for i in range(0, 128):
            buf += [0x00]

        buf[0] = 0x54
        buf[1] = 0x00
        buf[2] = 0x01
        buf[127] = 0x55

        return buf


class UwbDemarcate(UwbMsg):
    msg_id = UWB_MSG_ID_DEMARCATE
    msg_len = 128

    def __init__(self, uwb_state):
        if not isinstance(uwb_state, UwbState):
            raise UwbError("UwbSwitchMode location mode error")
        self._uwb_state = uwb_state

    def pack(self):
        buf = self._uwb_state.buf

        buf[2] = 2
        buf[17] = (buf[17] & 0xF0) + 8

        sum_val = 0
        for i in range(UwbDemarcate.msg_len - 1):
            sum_val += buf[i]
        buf[127] = sum_val & 0xFF

        return buf


uwb_msg_list = [UwbHeartbeat, UwbLocationResponse, UwbState]


class Uwb:

    @staticmethod
    def is_uwb_msg(buf):
        """
        return msg len while buf is belong to uwbmsg
        """
        for msgclass in uwb_msg_list:
            if buf[0] == msgclass.msg_header and buf[1] == msgclass.msg_function_mark:
                return msgclass.msg_len
        return -1

    @staticmethod
    def decode(buf):
        for msgclass in uwb_msg_list:
            try:
                msg = msgclass.decode(buf)
            except HeadError as e:
                print(f"Uwb decode HeadError: {e}")
                continue
            except UwbError as e:
                print(f"Uwb decode UwbError: {e}")
                raise
            return msg
        raise UwbError("Not an uwb msg")

    @staticmethod
    def uwb_switch_mode_encode(translate_mode, location_mode=0):
        return UwbSwitchMode(translate_mode, location_mode)

    @staticmethod
    def uwb_file_encode(buf, tar_type, tar_no):
        return UwbFile(buf, tar_type, tar_no)

    @staticmethod
    def uwb_get_state_encode():
        return UwbGetState()

    @staticmethod
    def uwb_demarcate_encode(uwb_state):
        return UwbDemarcate(uwb_state)


if __name__ == "__main__":
    import serial

    sum_val = 0
    srl = serial.Serial("COM6", 921600, timeout=2)

    uwb = Uwb()
    # msg = uwb.uwb_switch_mode_encode('DataTran')
    msg = uwb.uwb_switch_mode_encode("Location")
    buf = msg.pack()

    srl.write(buf)
    """
  while True:


      print('%02X ' % buf[0], end = '', flush = True)
  """
    recv = []
    while True:
        recv += srl.read(1)
        if len(recv) >= 128:
            msg = uwb.decode(recv)
            if msg is not None:
                print(msg.translate_mode)
                recv = []
