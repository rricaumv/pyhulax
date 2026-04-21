import threading

from . import buffer

# import buffer

from enum import Enum

ControllerType = Enum("ControllerType", ("NetworkController", "SerialController"))
RecvState = Enum("RecvState", ("Uninit", "Start", "Stop", "Delete", "DeleteDone"))


class CommunicationController:

    controller_type = None

    def __init__(self):
        self._buffer = buffer.Buffer()
        self._recv_state = RecvState.Uninit
        self._write_lock = threading.Lock()

    def connect(self, *args):
        pass

    def disconnect(self):
        pass

    def get_buffer(self):
        return self._buffer

    def send_buf(self, buf):
        pass

    def _recv_thread(self):
        pass

    def reset_timeout(self):
        pass

    def change_controller_type(self):
        pass
