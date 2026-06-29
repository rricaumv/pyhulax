"""Regression tests for per-drone identity isolation.

These cover the scoped refactor that stopped the SDK from sharing a single
module-level MAVLink encoder and a single global ``drone_id`` / ``plane_id``
across every connection. They run without hardware: commands are encoded and
decoded in-memory, and telemetry isolation is checked against the DataCenter.
"""

from pyhulax.fylo import mavlink
from pyhulax.fylo import config as fylo_config
from pyhulax.fylo.commandprocessor import CommandProcessorFactory
from pyhulax.system.command import Command, SysCommand
from pyhulax.system.datacenter import DataCenter


def _forward_command():
    return Command(
        SysCommand.S_Fly_Forward,
        {"plane_id": 1, "token": 5, "distance": 50, "led": 0, "speed": 100},
    )


def _encoder(component):
    return mavlink.MAVLink(None, src_system=1, src_component=component)


def _decode(buf):
    return mavlink.MAVLink(None).decode(bytearray(buf))


def test_per_connection_drone_id_is_stamped_into_command():
    """The threaded per-connection id wins over the legacy global."""
    fylo_config.drone_id = 99  # would previously leak into every command

    cp3 = CommandProcessorFactory.get_command_processor(
        _forward_command(), _encoder(10), drone_id=3
    )
    cp7 = CommandProcessorFactory.get_command_processor(
        _forward_command(), _encoder(11), drone_id=7
    )

    # FORMATION_CMD encodes the target plane id in start_id/end_id.
    assert _decode(cp3.get_buf()).start_id == 3
    assert _decode(cp7.get_buf()).start_id == 7


def test_encoders_have_independent_sequence_counters():
    """Each connection owns its encoder, so seq counters don't cross-talk."""
    mav_a, mav_b = _encoder(10), _encoder(11)
    assert mav_a is not mav_b

    seq_b_before = mav_b.seq
    for _ in range(3):
        CommandProcessorFactory.get_command_processor(
            _forward_command(), mav_a, drone_id=3
        ).get_buf()

    assert mav_b.seq == seq_b_before


def test_legacy_global_fallback_preserved():
    """Callers that pass no per-connection id keep the old global behaviour."""
    fylo_config.drone_id = 42
    cp = CommandProcessorFactory.get_command_processor(_forward_command())
    assert _decode(cp.get_buf()).start_id == 42


def test_command_identity_matches_official_app():
    """Commands must go out as src_system=255, src_component=bind_client.

    The drone only accepts commands from the ground-station identity it expects
    (the official app uses sys=255 and component=bind_client); a mismatch makes
    the drone silently ignore the command.
    """
    fylo_config.bind_client = 94
    # Encoder created the way TaskController creates it.
    mav = mavlink.MAVLink(None, src_system=255, src_component=2)
    cmd = Command(
        SysCommand.S_Fly_Takeoff,
        {"plane_id": 1, "token": 2, "height": 100, "led": 0, "flags": 0},
    )
    cp = CommandProcessorFactory.get_command_processor(cmd, mav, drone_id=1)
    msg = _decode(cp.get_buf())
    assert msg.cmd == 23  # ONE_KEY_TAKEOFF
    assert msg.get_header().srcSystem == 255
    assert msg.get_header().srcComponent == 94


def test_datacenter_telemetry_isolated_per_drone():
    """Telemetry stored under distinct ids does not cross-contaminate."""
    dc = DataCenter()
    dc.set_data("Plane", "heartbeat", "HB_3", 3)
    dc.set_data("Plane", "heartbeat", "HB_7", 7)
    dc.set_data("Plane", "heartbeat", "HB_LEGACY", 0)

    assert dc.get_data("Plane", "heartbeat", 3) == "HB_3"
    assert dc.get_data("Plane", "heartbeat", 7) == "HB_7"
    assert dc.get_data("Plane", "heartbeat") == "HB_LEGACY"
