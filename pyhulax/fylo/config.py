"""MAVLink protocol configuration.

Module-level variables for backward compatibility.
Values sourced from central config: pyhulax.config
"""

from __future__ import annotations

from pyhulax.config import DroneConfig, get_config

_cfg = get_config()

device = "hula"

serial_baudrate = _cfg.protocol.serial_baudrate

mavlink_system_id = _cfg.protocol.mavlink_system_id
mavlink_component_id = _cfg.protocol.mavlink_component_id
mavlink_component_file_id = _cfg.protocol.mavlink_component_file_id
bind_client = 255  # Set dynamically from local IP on drone network (192.168.100.x)
drone_reported_bind_client = None  # Last bind_client advertised by the drone (msg 232)
drone_id = None  # Set dynamically at runtime

# Command sending protocol configuration
# "tcp" = Send commands via TCP port 8888 (reliable, default)
# "udp" = Send commands via UDP port 8085 (lower latency, best-effort)
# Note: ACK reception always uses TCP regardless of send protocol
command_protocol = _cfg.protocol.command_protocol


def apply_runtime_config(runtime_config: DroneConfig) -> None:
    """Update legacy module-level config values from a runtime config."""
    global serial_baudrate
    global mavlink_system_id
    global mavlink_component_id
    global mavlink_component_file_id
    global bind_client
    global drone_reported_bind_client
    global drone_id
    global command_protocol

    serial_baudrate = runtime_config.protocol.serial_baudrate
    mavlink_system_id = runtime_config.protocol.mavlink_system_id
    mavlink_component_id = runtime_config.protocol.mavlink_component_id
    mavlink_component_file_id = runtime_config.protocol.mavlink_component_file_id
    command_protocol = runtime_config.protocol.command_protocol

    # These are still discovered at runtime per active session.
    bind_client = 255
    drone_reported_bind_client = None
    drone_id = None

    from pyhulax.system import system

    system.command_ip = runtime_config.network.drone_ip
    system.command_port = runtime_config.network.rtp_base_port
    system.serial_baudrate = serial_baudrate
    system.mavlink_system_id = mavlink_system_id
    system.mavlink_component_id = mavlink_component_id
    system.mavlink_component_file_id = mavlink_component_file_id

    try:
        from . import commandprocessor

        commandprocessor.refresh_runtime_config()
    except Exception:
        pass

    try:
        from . import stateprocessor

        stateprocessor.refresh_runtime_config(runtime_config)
    except Exception:
        pass
