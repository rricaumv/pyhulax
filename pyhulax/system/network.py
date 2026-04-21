import socket
import re
from socket import timeout
from enum import Enum

ConStatus = Enum("ConStatus", ("DISCON", "CON"))
Protocol = Enum("Protocol", ("TCP", "UDP"))


class Network:
    def __init__(self, ip, port):
        if not (
                isinstance(ip, str)
                and re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    ip,
                ) is not None
        ):
            raise ValueError
        if port < 1024 or port > 65535:
            raise ValueError

        self._timeout = None
        self._ip = ip
        self._port = port
        self._protocol = None
        self._sock = None
        self._connect_status = ConStatus.DISCON

    def __del__(self):
        self.network_deinit()

    def network_init(self):
        """
        Create socket and connect to the server
        """  
        pass

    def network_deinit(self):
        """
        Disconnect server connection and turn off the socket
        """
        if self._connect_status == ConStatus.CON:
            self._sock.close()
            self._connect_status = ConStatus.DISCON

    def send(self, buf):
        """
        Send buf to server
        """  
        pass

    def recv(self, size=1024):
        """
        Receive message from socket.
        """
        if self._connect_status == ConStatus.CON:
            try:
                return self._sock.recv(size)
            except timeout as e:
                print("Receive data timeout: %s" % e)
                raise
            except socket.error as e:
                print("Error receiving data: %s" % e)
                raise

    def set_recv_timeout(self, time):
        if not (isinstance(time, int) or isinstance(time, float) or time is None):
            raise ValueError

        if time == self._timeout:
            return

        self._timeout = time
        self._sock.settimeout(time)

    def reset_ip(self, ip):
        """
        Reset IP
        """
        if not (
                isinstance(ip, str)
                and re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    ip,
                ) is not None
        ):
            raise ValueError
        if ip == self._ip:
            return

        if self._connect_status == ConStatus.CON:
            self.network_deinit()
            self._ip = ip
            self.network_init()
        else:
            self._ip = ip

    def reset_port(self, port):
        """
        Reset server port
        """
        if port < 1024 or port > 65535:
            raise ValueError
        if port == self._port:
            return

        if self._connect_status == ConStatus.CON:
            self.network_deinit()
            self._port = port
            self.network_init()
        else:
            self._port = port


class TCPNetwork(Network):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        self._protocol = Protocol.TCP

    def network_init(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error as e:
            print("Error creating socket:%s" % e)
            raise

        try:
            self._sock.connect((self._ip, self._port))
        except socket.gaierror as e:
            print("Address-related error connecting to server: %s" % e)
            raise

        self._connect_status = ConStatus.CON

    def send(self, buf):
        if type(buf) == list:
            print("buf88888:", buf)
            buf = bytes(buf)
        return self._sock.send(buf)


class UDPNetwork(Network):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        self._protocol = Protocol.UDP

    def network_init(self):
        print("init")
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except socket.error as e:
            print("Error creating socket:%s" % e)
            raise
        print(self._ip, self._port)
        self._sock.bind((self._ip, self._port))
        self._connect_status = ConStatus.CON

    def send(self, buf):
        if type(buf) == list:
            print("buf99999:", buf)
            buf = bytes(buf)
        return self._sock.sendto(buf, (self._ip, self._port))


if __name__ == "__main__":
    print("test")
    network = UDPNetwork("192.168.0.109", 8400)
    # network = TCPNetwork('192.168.1.32', 7000)

    network.network_init()
    buf = network.recv(1024)
    print(buf)
    # network.reset_port(9000)
    network.send(b"hello")
    buf = network.recv(1024)
    print(buf)

    # network.reset_ip("192.168.1.180")
    network.send(b"hello")
    buf = network.recv(1024)
    print(buf)

    network.network_deinit()
