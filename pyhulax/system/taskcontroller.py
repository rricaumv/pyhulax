import os
import threading
import time
from pyhulax.config import DroneConfig, resolve_config
from . import system
from . import event
from ..system.buffer import *
from ..system.datacenter import *
from socket import *
from ..fylo import config

from ..fylo.taskprocessor import UserTask, SysTask, TaskProcessorFactory
from ..fylo.stateprocessor import StateProcessorFactory
from ..fylo.commandprocessor import CommandProcessorFactory
from ..fylo import mavlink
from ..fylo import uwb
from ..fylo import msganalyzer
from ..fylo import mavanalyzer
from ..system import system
from ..system import datacenter
from ..system.command import SysCommand
from ..logging.file_logger import FileLoggerMiddleware


class TaskController:
    def __init__(
        self,
        server_ip,
        runtime_config: DroneConfig | None = None,
        enable_file_logging: bool = True,
        log_dir: str = "logs",
        drone_id: int | None = None,
        source_ip: str | None = None,
    ):
        # self._communication_controller = communication_controller
        # self._buffer = self._communication_controller.get_buffer()
        # self._msganalyzer = msganalyzer.MsgAnalyzer(self._buffer)
        self._config = resolve_config(runtime_config)
        self.server_ip = server_ip or self._config.network.drone_ip
        self._controller_status = 1
        # Per-connection drone id. May be supplied explicitly (multi-drone) or
        # discovered from this connection's own telemetry stream at runtime.
        self._drone_id = drone_id
        self._plane_id = drone_id if drone_id is not None else 1
        self._token = None

        # Local IP to bind outbound sockets to. On a multi-homed host (more than
        # one interface on the drone's subnet) the default route may egress from
        # the wrong interface, so the drone never streams over the TCP
        # connection / never sees the client IP it expects. When set, the TCP
        # connection and UDP send sockets are bound to this address.
        self._source_ip = source_ip

        # Lightweight connection diagnostics so a failed connect() can report
        # exactly where the pipeline stalled (TCP connect / bytes / parsing).
        self._tcp_connected = False
        self._tcp_error: str | None = None
        self._rx_bytes = 0
        self._tcp_rx_bytes = 0
        self._udp_rx_bytes = 0
        self._udp_peer = None
        self._rx_msg_count = 0
        self._rx_msg_ids: dict[int, int] = {}

        # Per-connection MAVLink encoders. Each TaskController owns its own
        # encoder so the sequence counter and source component are not shared
        # across drones (the old module-level encoder in commandprocessor was
        # mutated by every connection).
        self._mavlink = mavlink.MAVLink(
            None,
            src_system=system.mavlink_system_id,
            src_component=system.mavlink_component_id,
        )
        self._mavlink_file = mavlink.MAVLink(
            None,
            src_system=system.mavlink_system_id,
            src_component=system.mavlink_component_file_id,
        )
        # self._dancefileanalyzer = dancefileanalyzer.DanceFileAnalyzer(
        #     os.path.abspath(os.path.join(os.path.dirname(__file__), '../dancefile')))
        self._event = event.Event()

        # File logging middleware for saving parsed messages
        self._file_logger = FileLoggerMiddleware(log_dir) if enable_file_logging else None

        self._udp_recive_msg_thread = threading.Thread(
            target=self.udp_recive_thread, name="UDP_RECIVE_THREAD"
        )
        self._udp_recive_msg_thread.setDaemon(True)
        self._udp_recive_msg_thread.start()

        self._tcp_server_thread = threading.Thread(
            target=self.tcp_server_thread, name="TCP_SERVER_THREAD"
        )
        self._tcp_server_thread.setDaemon(True)
        self._tcp_server_thread.start()

        # self._task_control_matrix = np.zeros((255, 5),int)  # colum[0]:   colum[1]:ip  colum[2]:dance updata colum[3]:tcp alive count
        # self._socket_buffer_list = [Buffer() for i in range(0, 255)]
        self._socket_buffer = Buffer()

        self._msganalyzer = mavanalyzer.MavAnalyzer(self._socket_buffer)

        self._msg_anlyzer_thread = threading.Thread(target=self.tcp_msg_anlyzer_thread)
        self._msg_anlyzer_thread.setDaemon(True)
        self._msg_anlyzer_thread.start()

        
        # self._plane_status_list=np.zeros((255,3),int) # [0]::0  1    [1]:ip [2]:tcp alive count

        self._datacenter = DataCenter()  #
        # self.lock = threading.Lock()

        # Initialize UDP command socket for protocol="udp" mode
        self.udp_command_socket = None
        if config.command_protocol == "udp":
            self._init_udp_command_socket()

    def create_task(self, utask, data=None, cycle=0):

        tp = TaskProcessorFactory.get_task_processor(self, utask, data)
        if cycle == 0:
            return tp.work()
        if cycle == 1:
            _thread = threading.Thread(target=self._work_thread, args=(tp.work,))
            _thread.setDaemon(True)
            _thread.start()
        elif cycle == 0:
            tp.work()
        elif cycle == 2:
            _thread = threading.Thread(target=tp.work)
            _thread.setDaemon(True)
            _thread.start()
            # _thread.join()

    def stop_all_task(self):
        self._controller_status = 0
        # self._msganalyzer.stop_get_msg()

        # Close file logger
        if self._file_logger is not None:
            self._file_logger.close()

        # Close UDP receive socket
        try:
            if hasattr(self, 'udp_recive_socket'):
                self.udp_recive_socket.close()
        except Exception:
            pass

        # Close TCP server socket
        try:
            if hasattr(self, 'server_socket'):
                self.server_socket.close()
        except Exception:
            pass

        # Close UDP broadcast socket
        try:
            if hasattr(self, 'udp_socket'):
                self.udp_socket.close()
        except Exception:
            pass

        # Close UDP command socket
        try:
            if hasattr(self, 'udp_command_socket') and self.udp_command_socket is not None:
                self.udp_command_socket.close()
        except Exception:
            pass

    def _init_udp_command_socket(self):
        """Initialize UDP socket for command sending."""
        try:
            self.udp_command_socket = socket(AF_INET, SOCK_DGRAM)
            self.udp_command_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
            if self._source_ip:
                try:
                    self.udp_command_socket.bind((self._source_ip, 0))
                except OSError as e:
                    print(f"warning: could not bind UDP command socket to {self._source_ip}: {e}")
            print(
                f"UDP command socket initialized for "
                f"{self.server_ip}:{self._config.network.udp_command_port}"
            )
        except Exception as e:
            print(f"Failed to initialize UDP command socket: {e}")
            self.udp_command_socket = None

    def send_app_heartbeat(self, user_mode: int = 1, dest=None) -> bool:
        """Send APP_HEARTBEAT message via UDP.

        The drone requires periodic heartbeats to accept certain control modes.
        user_mode determines the app's current control mode:
            0 = Other
            1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
            2 = Program (autonomous flight mode)
            3 = Battle
            4 = Formation

        Args:
            user_mode: App mode (0-4). Default 1 for manual flight.
            dest: Optional (ip, port) to send to. Defaults to the drone IP on
                the configured command port. During discovery this is used to
                reply to the exact address a beacon arrived from.

        Returns:
            True if message was sent successfully.
        """
        # Ensure UDP socket is initialized
        if self.udp_command_socket is None:
            self._init_udp_command_socket()

        if self.udp_command_socket is None:
            print("UDP command socket not available for heartbeat")
            return False

        if dest is None:
            dest = (self.server_ip, self._config.network.udp_command_port)

        try:
            # Create MAVLink instance for encoding
            # Use same system/component IDs as C# mobile app:
            # - sysid=255 (byte.MaxValue in C#)
            # - compid=bind_client (last byte of local IP)
            mav = mavlink.MAVLink(
                None, src_system=255, src_component=config.bind_client
            )

            # Create and pack APP_HEARTBEAT message
            msg = mavlink.MAVLink_app_heartbeat_message(user_mode)
            buf = msg.pack(mav)

            self.udp_command_socket.sendto(buf, dest)
            return True
        except Exception as e:
            print(f"Failed to send app heartbeat: {e}")
            return False

    def send_manual_control(self, x: int, y: int, z: int, r: int, target: int = 0, buttons: int = 0) -> bool:
        """Send MANUAL_CONTROL message via UDP port 8085.

        This provides direct joystick-style control with simultaneous
        position and yaw movement.

        Args:
            x: Pitch (forward/back), -1000 to +1000. +1000 = full forward.
            y: Roll (left/right), -1000 to +1000. +1000 = full right.
            z: Throttle (up/down), -1000 to +1000. +1000 = full up.
            r: Yaw (rotation), -1000 to +1000. +1000 = full CCW.
            target: Target system ID (default 0).
            buttons: Button bitmask (default 0).

        Returns:
            True if message was sent successfully.
        """
        # Ensure UDP socket is initialized
        if self.udp_command_socket is None:
            self._init_udp_command_socket()

        if self.udp_command_socket is None:
            print("UDP command socket not available for manual control")
            return False

        # Clamp values to valid range
        x = max(-1000, min(1000, int(x)))
        y = max(-1000, min(1000, int(y)))
        z = max(-1000, min(1000, int(z)))
        r = max(-1000, min(1000, int(r)))

        try:
            # Create MAVLink instance for encoding
            # Use same system/component IDs as C# mobile app:
            # - sysid=255 (byte.MaxValue in C#)
            # - compid=bind_client (last byte of local IP)
            mav = mavlink.MAVLink(
                None, src_system=255, src_component=config.bind_client
            )

            # Create and pack MANUAL_CONTROL message
            msg = mavlink.MAVLink_manual_control_message(target, x, y, z, r, buttons)
            buf = msg.pack(mav)

            # Send via UDP to port 8085
            self.udp_command_socket.sendto(
                buf,
                (self.server_ip, self._config.network.udp_command_port),
            )
            return True
        except Exception as e:
            print(f"Failed to send manual control: {e}")
            return False

    def _send_command(self, syscmd, boradcast=False):

        cp = CommandProcessorFactory.get_command_processor(
            syscmd, self._mavlink, self._drone_id
        )
        buf = cp.get_buf()

        if buf != None and boradcast == False:
            # Select protocol based on config
            protocol = config.command_protocol

            if protocol == "udp":
                # Send via UDP port 8085
                if self.udp_command_socket is None:
                    self._init_udp_command_socket()

                if self.udp_command_socket is None:
                    print("UDP command socket not available, falling back to TCP")
                    protocol = "tcp"
                else:
                    try:
                        self.udp_command_socket.sendto(
                            buf,
                            (self.server_ip, self._config.network.udp_command_port),
                        )
                        # print(f"Sent {len(buf)} bytes via UDP to {self.server_ip}:8085")
                    except Exception as e:
                        print(f"UDP send failed: {e}, falling back to TCP")
                        protocol = "tcp"

            if protocol == "tcp":
                # Send via TCP port 8888 (default)
                client_socket = self.server_socket
                if client_socket is None:
                    print("clinet_socket is None")
                    return -1

                client_socket.sendall(buf)
                # print(ret,'',len(buf))

    
    
    
    
    
    
    
    

    def connection_diagnostics(self) -> dict:
        """Snapshot of the receive pipeline state, for connect diagnostics."""
        return {
            "tcp_connected": self._tcp_connected,
            "tcp_error": self._tcp_error,
            "rx_bytes": self._rx_bytes,
            "tcp_rx_bytes": self._tcp_rx_bytes,
            "udp_rx_bytes": self._udp_rx_bytes,
            "udp_peer": self._udp_peer,
            "rx_msg_count": self._rx_msg_count,
            "rx_msg_ids": dict(sorted(self._rx_msg_ids.items())),
            "drone_id": self._drone_id,
        }

    def send_app_heartbeat_from_status(self, user_mode: int = 2, dest=None) -> bool:
        """Send APP_HEARTBEAT *from* the status socket (bound to udp_status_port).

        The drone advertises and streams over UDP, and these protocols commonly
        key on / reply to the app's status port. Sending the heartbeat from the
        same socket we listen on (rather than an ephemeral send port) makes the
        app's source port the one the drone expects, and ensures any reply lands
        on the socket we're already receiving from.
        """
        sock = getattr(self, "udp_recive_socket", None)
        if sock is None:
            return False
        if dest is None:
            dest = (self.server_ip, self._config.network.udp_command_port)
        try:
            mav = mavlink.MAVLink(
                None, src_system=255, src_component=config.bind_client
            )
            msg = mavlink.MAVLink_app_heartbeat_message(user_mode)
            buf = msg.pack(mav)
            sock.sendto(buf, dest)
            return True
        except Exception:
            return False

    def send_app_heartbeat_tcp(self, user_mode: int = 2) -> bool:
        """Send an APP_HEARTBEAT over the established TCP command connection.

        Some drone firmware expects the ground-station heartbeat on the same
        TCP channel it streams over, not only the UDP broadcast/unicast path.
        """
        sock = getattr(self, "server_socket", None)
        if sock is None or not self._tcp_connected:
            return False
        try:
            mav = mavlink.MAVLink(
                None, src_system=255, src_component=config.bind_client
            )
            msg = mavlink.MAVLink_app_heartbeat_message(user_mode)
            buf = msg.pack(mav)
            sock.sendall(buf)
            return True
        except OSError:
            # The drone commonly closes the TCP channel while unbound; that's
            # expected and informative (the session runs over UDP), so stay
            # quiet here rather than spamming per-attempt.
            return False
        except Exception:
            return False

    def get_drone_id(self):
        """Return the drone id bound to this connection, if known.

        The id is either supplied explicitly at construction or discovered from
        this connection's own telemetry stream (heartbeat / plane status).
        """
        return self._drone_id

    def _capture_drone_id(self, msg):
        """Learn this connection's drone id from its own message stream.

        Only the messages that carry an authoritative identity are used:
        REPORT_STATS (207, ``drone_id``) and PLANE_STATUS (231, ``plane_id``).
        Once learned, the id is sticky for the lifetime of the connection.
        """
        if self._drone_id is not None:
            return
        try:
            msg_id = msg.get_msg_id()
        except Exception:
            return
        candidate = None
        if msg_id == 207:
            candidate = getattr(msg, "drone_id", None)
        elif msg_id == 231:
            candidate = getattr(msg, "plane_id", None)
        if candidate:
            self._drone_id = candidate
            self._plane_id = candidate

    def _mirror_telemetry(self, msg):
        """Mirror per-drone telemetry into the DataCenter keyed by drone id.

        The shared state processors store telemetry under the legacy slot
        ``id=0`` of the (singleton) DataCenter, which cross-contaminates across
        drones. Because each TaskController's socket only ever receives its own
        drone's messages, we additionally store the relevant telemetry under
        this connection's real drone id so callers can read it unambiguously.
        """
        if self._drone_id is None:
            return
        try:
            msg_id = msg.get_msg_id()
        except Exception:
            return
        if msg_id == 206:
            self._datacenter.set_data("Plane", "flight_data", msg, self._drone_id)
        elif msg_id == 207:
            self._datacenter.set_data("Plane", "heartbeat", msg, self._drone_id)

    def _wait_state(self, sysstate, timeout):
        # self._dispatcher
        _state = self._event.wait_for_signal(sysstate, timeout)
        if _state == None:
            return None
        else:
            return _state.get_data()

    def _work_thread(self, work):
        while self._controller_status:
            work()

    def _detach_work_thread(self, work):
        print("detach_work_thread")
        work()

    def _data_analyzer_thread(self):
        while self._controller_status:
            msg = self._msganalyzer.get_msg()
            _state_processor = StateProcessorFactory.get_state_processor(msg)
            if _state_processor == None:
                # print('get state processor error')
                continue
            _state = _state_processor.get_state()

            if _state == None:
                continue
            if isinstance(_state, list):
                for st in _state:
                    self._event.create_signal(_state.get_state(), st)
                    time.sleep(0.01)
            else:
                self._event.create_signal(_state.get_state(), _state)
                time.sleep(0.01)

    def tcp_server_thread(self):

        port = self._config.network.tcp_port
        host = self.server_ip
        print(f"{host}:{port}")
        # host = "127.0.0.1"
        # port = 5762
        # print(f'{host}:{port}')
        client_address = (host, port)
        self.server_socket = socket(AF_INET, SOCK_STREAM)
        try:
            # Bind to the chosen local interface so the drone sees (and streams
            # back over) a connection from the client IP it expects. Critical on
            # multi-homed hosts where the default route uses the wrong NIC.
            if self._source_ip:
                try:
                    self.server_socket.bind((self._source_ip, 0))
                except OSError as e:
                    print(f"warning: could not bind TCP to {self._source_ip}: {e}")
            self.server_socket.settimeout(self._config.timeouts.tcp_connect_timeout_sec)
            self.server_socket.connect(client_address)
            self.server_socket.settimeout(
                self._config.timeouts.tcp_recv_timeout_sec
            )  # Set timeout for recv after connect
            self._tcp_connected = True

            while self._controller_status:
                try:
                    recv_data = self.server_socket.recv(2048)
                    if not recv_data:
                        break  # Connection closed
                    self._rx_bytes += len(recv_data)
                    self._tcp_rx_bytes += len(recv_data)
                    self._socket_buffer.set_data(recv_data)
                except timeout:
                    continue  # Check _controller_status and loop again
                except OSError:
                    break  # Socket was closed

        except (OSError, Exception) as e:
            self._tcp_error = f"{type(e).__name__}: {e}"
            print(f"TCP connection error {host}:{port}: {e}")

        finally:
            try:
                self.server_socket.close()
            except:
                pass

    def udp_recive_thread(self):
        # PYHULA_UDP_PATCH_APPLIED https://github.com/janisgra/pyhula-install-wrapper/blob/2b1a1c6d47346ddfe23de6c818437cdc0b2cf7b7/pyhula_patcher.py#L160
        self.listen_port = self._config.network.udp_status_port
        self.udp_recive_socket = socket(AF_INET, SOCK_DGRAM)
        self.udp_recive_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.udp_recive_socket.settimeout(self._config.timeouts.udp_timeout_sec)
        # self.udp_recive_dest = (self.server_ip, self.listen_port)
        self.udp_recive_dest = ('0.0.0.0', self.listen_port)

        try:
            self.udp_recive_socket.bind(self.udp_recive_dest)
        except OSError as e:
            # Address not valid or already in use
            if e.errno in (99, 98):  # 99=EADDRNOTAVAIL, 98=EADDRINUSE
                # Try binding to localhost instead
                try:
                    self.udp_recive_socket.bind(("0.0.0.0", self.listen_port))
                    print(f"UDP bound to localhost:{self.listen_port} (fallback)")
                except:
                    # Try any available port
                    self.udp_recive_socket.bind(("0.0.0.0", 0))
                    print(
                        f"UDP bound to localhost:{self.udp_recive_socket.getsockname()[1]} (auto-assigned)"
                    )
            else:
                raise e

        while self._controller_status:
            try:
                recv_data, peer = self.udp_recive_socket.recvfrom(2048)
                self._rx_bytes += len(recv_data)
                self._udp_rx_bytes += len(recv_data)
                self._udp_peer = peer
                self._socket_buffer.set_data(recv_data)
            except timeout:
                continue  # Check _controller_status and loop again
            except OSError:
                break  # Socket was closed

    def anlyzer_drone_id(self, buffer):
        # print(type(buffer))
        mavMsg = mavlink.MAVLink(None)
        if buffer[0] == 0xFE:
            ba = bytearray(buffer)
            try:
                msg = mavMsg.decode(ba)
            except mavlink.MAVError as e:
                print(e)
                return -1
        return msg.drone_id

    def tcp_msg_anlyzer_thread(self):

        while self._controller_status:
            msg = self._msganalyzer.get_msg()

            # Record which message ids arrive (diagnostics for connect issues).
            try:
                _mid = msg.get_msg_id()
                self._rx_msg_count += 1
                self._rx_msg_ids[_mid] = self._rx_msg_ids.get(_mid, 0) + 1
            except Exception:
                pass

            # Learn this connection's drone id and mirror its telemetry into a
            # per-drone DataCenter slot before running the shared state
            # processors (which only write the legacy id=0 slot).
            self._capture_drone_id(msg)
            self._mirror_telemetry(msg)

            # Log ALL messages first, even if no state processor exists
            # This ensures we capture high-frequency streams like LOCAL_POSITION (msg 72)
            if self._file_logger is not None:
                # We'll update state after processor runs (if available)
                self._file_logger.log_message(msg, None)

            _state_processor = StateProcessorFactory.get_state_processor(msg)

            if _state_processor == None:
                continue

            _state = _state_processor.get_state()

            if _state == None:
                continue
            if isinstance(_state, list):
                for st in _state:
                    self._event.create_signal(_state.get_state(), st)
                    time.sleep(0.01)
            else:
                self._event.create_signal(_state.get_state(), _state)
                time.sleep(0.01)

    def udp_Rtp(self):
        """Receive and display RTP video stream from drone using PyAV."""
        import av
        import cv2
        import tempfile
        import os

        # Port is based on plane_id: 9000 + plane_id * 2. Prefer this
        # connection's own drone id; fall back to the legacy global.
        drone_id = self._drone_id
        if drone_id is None:
            drone_id = config.drone_id if config.drone_id is not None else 1
        port = self._config.network.rtp_base_port + drone_id * 2

        print(f"Starting video stream on UDP port {port}")

        # Create SDP file to tell FFmpeg the stream format (H.264 over RTP)
        # Get local IP
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((self.server_ip, 80))
            local_ip = s.getsockname()[0]
        except:
            local_ip = "0.0.0.0"
        finally:
            s.close()

        # SDP format matching C# app - payload type 98
        sdp_content = (
            f"v=0\n"
            f"o=- 0 0 IN IP4 0.0.0.0\n"
            f"s=DroneStream\n"
            f"c=IN IP4 0.0.0.0\n"
            f"t=0 0\n"
            f"m=video {port} RTP/AVP 98\n"
            f"a=rtpmap:98 H264/90000\n"
        )
        print(f"SDP:\n{sdp_content}")

        # Write SDP to temp file
        fd, sdp_path = tempfile.mkstemp(suffix='.sdp', prefix='drone_video_')
        with os.fdopen(fd, 'w') as f:
            f.write(sdp_content)

        print(f"SDP file: {sdp_path}")
        print(f"Waiting for video data...")

        options = {
            'protocol_whitelist': 'file,rtp,udp',
            'fflags': 'nobuffer',
            'flags': 'low_delay',
            'reorder_queue_size': '0',
            'max_delay': '0',
        }

        try:
            container = av.open(sdp_path, format='sdp', options=options, timeout=30.0)
            print(f"Video stream opened, waiting for first frame...")

            cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

            frame_count = 0
            for frame in container.decode(video=0):
                if not self._controller_status:
                    print("Controller stopped")
                    break

                frame_count += 1
                if frame_count == 1:
                    print(f"First frame received: {frame.width}x{frame.height}")

                img = frame.to_ndarray(format='bgr24')
                cv2.imshow("Video", img)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            print(f"Total frames decoded: {frame_count}")

        except Exception as e:
            print(f"Video stream error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            cv2.destroyAllWindows()
            # Cleanup SDP file
            try:
                os.unlink(sdp_path)
            except:
                pass
            print("Video stream closed")

    

    def udp_rtp_udp_recive_thread(self):

        self._rtp_recive_msg_thread = threading.Thread(
            target=self.udp_Rtp, name="UDP_RECIVE_THREAD"
        )
        self._rtp_recive_msg_thread.setDaemon(True)
        self._rtp_recive_msg_thread.start()

    # def udp_broadcast_thread(self):
    #     server_list = self.server_ip.split(".")
    #     _mavlink = mavlink.MAVLink(
    #         None, srcSystem=system.mavlink_system_id, srcComponent=config.bind_client
    #     )
    #     broad_cast_ip = (
    #         server_list[0] + "." + server_list[1] + "." + server_list[2] + "." + "255"
    #     )
    #     print(broad_cast_ip, self.server_ip)
    #     self.udp_dest = (broad_cast_ip, 8085)
    #     self.udp_socket = socket(AF_INET, SOCK_DGRAM)
    #     self.udp_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

    #     while self._controller_status:
    #         msg = _mavlink.app_heartbeat_encode(2)
    #         _mavlink.send(msg)
    #         send_buf = msg.pack(_mavlink)
    #         ret = self.udp_socket.sendto(send_buf, self.udp_dest)
    #         time.sleep(1)
    #     self.udp_socket.close()
    def set_app_mode(self, mode: int) -> None:
        """Set the app mode for heartbeat messages.

        This affects the user_mode sent in APP_HEARTBEAT messages.
        Mode values:
            0 = Other
            1 = Aerial (manual flight mode) - required for MANUAL_CONTROL
            2 = Program (autonomous flight mode) - default
            3 = Battle
            4 = Formation

        Args:
            mode: App mode (0-4).
        """
        self._app_mode = mode

    def get_app_mode(self) -> int:
        """Get the current app mode for heartbeat messages."""
        return getattr(self, '_app_mode', 2)  # Default to Program mode

    def udp_broadcast_thread(self):
        server_list = self.server_ip.split(".")

        # Use same system/component IDs as C# mobile app:
        # - sysid=255 (byte.MaxValue in C#)
        # - compid=bind_client (last byte of local IP)
        _mavlink = mavlink.MAVLink(
            None,
            src_system=255,
            src_component=config.bind_client,
        )

        broad_cast_ip = (
            f"{server_list[0]}.{server_list[1]}.{server_list[2]}.255"
        )

        print(broad_cast_ip, self.server_ip)

        self.udp_dest = (broad_cast_ip, self._config.network.udp_command_port)

        self.udp_socket = socket(AF_INET, SOCK_DGRAM)
        self.udp_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        # this is the important part for non-root broadcast:
        self.udp_socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        if self._source_ip:
            try:
                self.udp_socket.bind((self._source_ip, 0))
            except OSError as e:
                print(f"warning: could not bind broadcast socket to {self._source_ip}: {e}")

        try:
            while self._controller_status:
                # Use configurable app mode (default 2 = Program)
                mode = self.get_app_mode()
                msg = _mavlink.app_heartbeat_encode(mode)
                _mavlink.send(msg)
                send_buf = msg.pack(_mavlink)

                ret = self.udp_socket.sendto(send_buf, self.udp_dest)
                # optionally check ret here

                time.sleep(1)
        finally:
            self.udp_socket.close()
    def udp_heartbeat_send_thread(self):

        self._udp_broadcast_thread = threading.Thread(
            target=self.udp_broadcast_thread, name="UDP_BROADCAST_THREAD"
        )
        self._udp_broadcast_thread.setDaemon(True)
        self._udp_broadcast_thread.start()
