from . import mavlink


class MavAnalyzer:
    def __init__(self, buffer):
        self._msg_flag = 1
        self._buffer = buffer
        self._mav = mavlink.MAVLink(None)

    def stop_get_msg(self):
        self._msg_flag = 0
        self._buffer.set_timeout(0)
        self._buffer.wake_up()

    def get_msg(self):
        while self._msg_flag:
            buf = self._buffer.get_data_try(5)
            # print('buf000000:',buf)
            if buf is None:
                continue
            if buf[0] == 0xFE:
                buf = self._buffer.get_data_try(buf[1] + 8)
                # print("buf2222222:", buf)
                if buf is None:
                    continue
                ba = bytearray(buf)
                try:
                    msg = self._mav.decode(ba)
                    self._buffer.get_data_confirm()

                    # print("get mavlink msg")
                    # print(type(msg))
                    return msg
                except mavlink.MAVError as e:
                    print(e)
                    pass
