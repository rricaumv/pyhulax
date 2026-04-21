import serial
import time

# import serial.tools.list_ports

from .communicationcontroller import *


class SerialController(CommunicationController):

    controller_type = ControllerType.SerialController

    def __init__(self):
        super().__init__()
        self._serial = None

    def __del__(self):
        self.disconnect()

    @staticmethod
    def get_list():
        port_list = list(serial.tools.list_ports.comports())
        device_list = [p.device for p in port_list]
        return device_list

    def connect(self, port, baudrate=921600, timeout=2):
        list(serial.tools.list_ports.comports())
        if isinstance(port, int) and ("COM" + str(port) in SerialController.get_list()):
            if isinstance(baudrate, int) and 0 < baudrate < 1500000:
                if (
                    isinstance(timeout, int) or isinstance(timeout, float)
                ) and timeout > 0:
                    self._port = port
                    self._baudrate = baudrate
                    self._timeout = timeout
                    self._serial = serial.Serial(
                        "COM" + str(port), baudrate, timeout=timeout
                    )
                    self._thread = threading.Thread(
                        target=self._recv_thread, name="SerialRecvThread"
                    )
                    self._thread.setDaemon(True)
                    self._thread.start()
                    return True
        return False

    def connect_by_port_name(
        self,
        port_name=("Silicon Laboratories", "Silicon Labs"),
        baudrate=921600,
        timeout=2,
    ):
        if isinstance(baudrate, int) and 0 < baudrate < 1500000:
            if (isinstance(timeout, int) or isinstance(timeout, float)) and timeout > 0:
                port_list = list(serial.tools.list_ports.comports())
                for port in port_list:
                    for name in port_name:
                        if port.manufacturer == name:
                            self._port = int(port.device[3:4])
                            self._baudrate = baudrate
                            self._timeout = timeout
                            self._serial = serial.Serial(
                                "COM" + str(self._port), baudrate, timeout=timeout
                            )
                            self._thread = threading.Thread(
                                target=self._recv_thread, name="SerialRecvThread"
                            )
                            self._thread.setDaemon(True)
                            self._thread.start()
                            return True
        return False

    def disconnect(self):
        self._recv_state = RecvState.Delete
        self._thread = threading.Thread(
            target=self._disconnect_thread, name="SerialDisconnectThread"
        )
        self._thread.setDaemon(True)
        self._thread.start()

    """






  """

    def send_buf(self, buf):
        if self._serial is None:
            return -1
        self._write_lock.acquire()
        ret = self._serial.write(buf)
        self._write_lock.release()
        return ret

    def reset_port(self, port):
        if self._serial is None:
            return False

        port_list = list(serial.tools.list_ports.comports())
        if isinstance(port, int) and (
            len([p for p in port_list if p.device == "COM" + str(port)]) == 1
        ):
            self._recv_state = RecvState.Stop
            self._port = port
            self._serial.port = "COM" + str(port)
            self._recv_state = RecvState.Start

        return True

    def reset_baudrate(self, baudrate):
        if self._serial is None:
            return False

        if isinstance(baudrate, int) and 0 < baudrate < 1500000:
            self._recv_state = RecvState.Stop
            self._baudrate = baudrate
            self._serial.baudrate = baudrate
            self._recv_state = RecvState.Start

        return True

    def reset_timeout(self, time):
        if self._serial is None:
            return False

        if (isinstance(time, int) or isinstance(time, float)) and time > 0:
            self._recv_state = RecvState.Stop
            self._timeout = time
            self._serial.timeout = time
            self._recv_state = RecvState.Start

        return True

    def _recv_thread(self):
        self._recv_state = RecvState.Start
        while self._recv_state != RecvState.Delete:
            if self._recv_state == RecvState.Start:
                # """
                num = self._serial.inWaiting()
                if not num == 0:
                    self._buffer.set_data(self._serial.read(num))
                else:
                    time.sleep(0.1)
                # """
                # self._buffer.set_data(self._serial.read(1))
            elif self._recv_state == RecvState.Stop:
                time.sleep(2)
        self._recv_state = RecvState.DeleteDone

    def _disconnect_thread(self):
        while self._recv_state != RecvState.DeleteDone:
            pass
        if self._serial is not None:
            self._serial.close()
            self._serial = None
            self._buffer.clean_buf()
        print("communication disconnect")


if __name__ == "__main__":
    pl = SerialController.get_list()
    print(type(pl[0]))
    if len(pl) == 0:
        print("pl is none")
    else:
        for i in range(0, len(pl)):
            print(pl[i])

    sc = SerialController()
    buffer = sc.get_buffer()
    buffer.set_timeout(2)
    sc.serial_init(39)
    sc.reset_baudrate(115200)
    sc.send_buf(b"hello world")

    i = 0
    while True:
        buf = buffer.get_data(1)
        print(buf)

    print(buffer.get_data(100))
    print(buffer.get_data(100))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
