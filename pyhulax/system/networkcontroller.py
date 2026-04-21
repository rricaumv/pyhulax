import time

from . import network
from .communicationcontroller import *


class NetworkController(CommunicationController):
    controller_type = ControllerType.NetworkController

    def __init__(self):
        super().__init__()
        self._network = None

    def __del__(self):
        self.disconnect()

    @staticmethod
    def get_list():
        port_list = list(serial.tools.list_ports.comports())
        device_list = [p.device for p in port_list]
        return device_list

    def connect(self, ip="192.168.100.1", port=8400, protocol="Udp", timeout=1):
        # def connect(self, ip = '127.0.0.1', port = 7777, protocol = 'Udp', timeout = 1):
        try:
            if protocol == "Tcp":
                self._network = network.TCPNetwork(ip, port)
            elif protocol == "Udp":
                self._network = network.UDPNetwork(ip, port)
        except ValueError as e:
            print(f"network init error: {e}")
            return False
        print("protocol222222", protocol)
        self._network.network_init()
        self._network.set_recv_timeout(timeout)
        self._ip = ip
        self._port = port
        self._timeout = timeout
        self._protocol = protocol
        self._thread = threading.Thread(
            target=self._recv_thread, name="NetworkRecvThread"
        )
        self._thread.setDaemon(True)
        self._thread.start()
        return True

    def disconnect(self):
        self._recv_state = RecvState.Delete
        self._thread = threading.Thread(
            target=self._disconnect_thread, name="NetworkDisconnectThread"
        )
        self._thread.setDaemon(True)
        self._thread.start()

    """















    """

    def send_buf(self, buf):
        if self._network is None or buf is None:
            return -1

        self._write_lock.acquire()
        ret = self._network.send(buf)
        self._write_lock.release()
        return ret

    def reset_port(self, port):
        if self._network is None:
            return False
        ret = True
        self._recv_state = RecvState.Stop
        time.sleep(self._timeout + 0.1)
        try:
            self._network.reset_port(port)
        except ValueError as e:
            print(f"reset port error: {e}")
            ret = False
        if ret:
            self._port = port
        self._recv_state = RecvState.Start
        return ret

    def reset_ip(self, ip):
        if self._network is None:
            return False

        ret = True
        self._recv_state = RecvState.Stop
        time.sleep(self._timeout + 0.1)
        try:
            self._network.reset_ip(ip)
        except ValueError as e:
            print(f"reset ip error: {e}")
            ret = False
        if ret:
            self._ip = ip
        self._recv_state = RecvState.Start
        return ret

    def reset_timeout(self, time):
        if self._network is None:
            return False

        if (isinstance(time, int) or isinstance(time, float)) and time > 0:
            self._recv_state = RecvState.Stop
            try:
                self._network.set_recv_timeout(time)
            except ValueError as e:
                print(f"set timeout error: {e}")
                ret = False
            if ret:
                self._timeout = time
            self._recv_state = RecvState.Start
            return False

    def _recv_thread(self):
        self._recv_state = RecvState.Start
        while self._recv_state != RecvState.Delete:
            if self._recv_state == RecvState.Start:
                try:
                    self._buffer.set_data(self._network.recv(1024))
                except network.timeout as e:
                    print(f"recv timeout: {e}")
                    continue
            elif self._recv_state == RecvState.Stop:
                time.sleep(2)
        self._recv_state = RecvState.DeleteDone

    def _disconnect_thread(self):
        while self._recv_state != RecvState.DeleteDone:
            time.sleep(0.1)
        if self._network is not None:
            self._network.network_deinit()
            self._network = None
            self._buffer.clean_buf()
        print("communication disconnect")


if __name__ == "__main__":

    nc = NetworkController()
    buffer = nc.get_buffer()
    buffer.set_timeout(2)
    nc.network_init()
    nc.send_buf(b"hello world")

    nc.reset_port(8000)

    nc.send_buf(b"hello world")

    i = 0
    while True:
        buf = buffer.get_data(1)
        print(str(buf))

    print(buffer.get_data(100))
    print(buffer.get_data(100))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
    print(buffer.get_data(10))
