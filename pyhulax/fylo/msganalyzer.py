from . import config
from . import mavlink
from . import uwb


class MsgAnalyzer:
    def __init__(self, buffer):
        self._msg_flag = 1
        self._buffer = buffer
        self._uwb = uwb.Uwb()
        self._mav = mavlink.MAVLink(
            None, config.mavlink_system_id, config.mavlink_component_id
        )
        self._mav_file = mavlink.MAVLink(
            None, config.mavlink_system_id, config.mavlink_component_file_id
        )

    def stop_get_msg(self):
        self._msg_flag = 0
        self._buffer.set_timeout(0)
        self._buffer.wake_up()

    def get_msg(self):
        while self._msg_flag:
            buf = self._buffer.get_data_try(5)
            print(buf)
            if buf is None:
                continue
            if buf[0] == 0xFE:
                if (
                        buf[3] == config.mavlink_system_id
                        and buf[4] == config.mavlink_component_id
                ):
                    buf = self._buffer.get_data_try(buf[1] + 8)
                    if buf is None:
                        continue
                    ba = bytearray(buf)
                    try:
                        msg = self._mav.decode(ba)
                        self._buffer.get_data_confirm()
                        # print('get mavlink msg')
                        # print(type(msg))
                        return msg
                    except mavlink.MAVError as e:
                        print(e)
                        pass

                elif (
                        buf[3] == config.mavlink_system_id
                        and buf[4] == config.mavlink_component_file_id
                ):
                    buf = self._buffer.get_data_try(buf[1] + 8)
                    if buf is None:
                        continue
                    ba = bytearray(buf)
                    try:
                        msg = self._mav_file.decode(ba)
                        self._buffer.get_data_confirm()
                        # print('get mavlink msg')
                        # print(type(msg))
                        return msg
                    except mavlink.MAVError as e:
                        print(e)
                        pass

            msg_len = self._uwb.is_uwb_msg(buf)
            if msg_len > 0:
                buf = self._buffer.get_data_try(msg_len)
                if buf is None:
                    continue
                # print(buf)
                try:
                    msg = self._uwb.decode(buf)
                    self._buffer.get_data_confirm()
                    # print('get uwb msg')
                    # print(type(msg))
                    return msg
                except uwb.UwbError as e:
                    print(e)
                    pass

            # buf = self._buffer.get_data(1)
            # print(buf)
            # print('error %02X ' % buf[0], end = '', flush = True)
