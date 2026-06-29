#!/usr/bin/env python3
"""Takeoff / hover / land demo for one and two drones.

This script deliberately uses the *forked and modified* ``pyhulax`` package
that lives in this repository rather than any version installed in
site-packages. The bootstrap below inserts the repo root at the front of
``sys.path`` and verifies the import actually resolved to this checkout.

The two-drone path exercises the per-connection drone identity work: each
drone gets its own ``DroneAPI`` (its own ``Controlserver`` / ``TaskController``,
its own MAVLink encoder) and an explicit ``drone_id`` so commands and telemetry
stay scoped to the right aircraft even while both fly at once.

Usage:
    # Single drone (default IP 192.168.100.1)
    python examples/takeoff_hover_land_demo.py one

    # Two drones flying concurrently
    python examples/takeoff_hover_land_demo.py two \
        --ips 192.168.100.1 192.168.100.2 --ids 1 2

    # Validate wiring without touching any hardware (no connect/flight)
    python examples/takeoff_hover_land_demo.py two --check
"""

from __future__ import annotations

# --------------------------------------------------------------------------- #
# Bootstrap: force-load the forked pyhulax from this repo, not the installed one
# --------------------------------------------------------------------------- #
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import pyhulax  # noqa: E402

_resolved = os.path.dirname(os.path.abspath(pyhulax.__file__))
_expected = os.path.join(_REPO_ROOT, "pyhulax")
if os.path.normcase(_resolved) != os.path.normcase(_expected):
    raise SystemExit(
        "Refusing to run: 'pyhulax' resolved to an unexpected location.\n"
        f"  loaded from: {_resolved}\n"
        f"  expected:    {_expected}\n"
        "Make sure the installed pyhulax is not shadowing the repo copy."
    )

import argparse  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402

from pyhulax import DroneAPI, DroneConfig, NetworkConfig, LEDConfig  # noqa: E402
from pyhulax.core import LEDMode  # noqa: E402
from pyhulax.core.exceptions import DroneError, DroneConnectionError  # noqa: E402


# --------------------------------------------------------------------------- #
# Flight sequence
# --------------------------------------------------------------------------- #
def build_drone(ip: str, drone_id: int | None) -> DroneAPI:
    """Create a DroneAPI bound to one drone's IP and (optionally) explicit id.

    Passing ``drone_id`` pins this connection's identity up front, which removes
    the brief pre-discovery window where telemetry could otherwise read the
    legacy shared slot. For multi-drone you almost always want to set it.
    """
    config = DroneConfig(network=NetworkConfig(drone_ip=ip))
    return DroneAPI(config=config, drone_id=drone_id)


def takeoff_hover_land(
    drone: DroneAPI,
    *,
    label: str,
    ip: str,
    height_cm: int,
    hover_seconds: float,
    connect_timeout: float = 8.0,
) -> None:
    """Run a single drone through connect -> takeoff -> hover -> land."""
    led = LEDConfig(r=0, g=255, b=0, mode=LEDMode.SOLID)

    def log(msg: str) -> None:
        print(f"[{label}] {msg}", flush=True)

    log(f"connecting to {ip} (timeout {connect_timeout:.0f}s) ...")
    try:
        drone.connect(ip, timeout=connect_timeout)
    except DroneConnectionError as exc:
        log(f"CONNECT FAILED: {exc}")
        log("troubleshooting:")
        log(f"  1. Confirm this machine is on the drone's WiFi and can reach {ip}")
        log("  2. Confirm the drone is powered on and finished booting")
        log("  3. Try a longer --connect-timeout (e.g. 15)")
        raise
    try:
        log(f"connected (drone_id={drone.get_drone_id()}), battery={_safe_battery(drone)}%")

        log(f"takeoff to {height_cm} cm")
        drone.takeoff(height_cm=height_cm, led=led)

        log(f"hover for {hover_seconds:.0f}s")
        drone.hover(hover_seconds, led=led)

        log("landing")
        drone.land(led=led)
        log("landed")
    finally:
        # Best-effort safety: try to land if we bailed out mid-flight, then
        # always release the connection/threads.
        try:
            if drone.is_connected:
                drone.land()
        except DroneError:
            pass
        drone.disconnect()
        log("disconnected")


def _safe_battery(drone: DroneAPI) -> int | str:
    try:
        return drone.get_battery()
    except DroneError:
        return "?"


# --------------------------------------------------------------------------- #
# Demos
# --------------------------------------------------------------------------- #
def _warn_if_zero_id(ids: list[int]) -> None:
    if any(i == 0 for i in ids):
        print("NOTE: drone_id 0 is unusual - hula aircraft are typically id>=1, "
              "and the firmware may ignore commands addressed to plane_id 0. "
              "Pass --ids with the real drone id(s) for actual flight.")


def demo_one(
    ip: str,
    drone_id: int | None,
    height_cm: int,
    hover_seconds: float,
    connect_timeout: float,
) -> None:
    print("=== Single-drone demo ===")
    _warn_if_zero_id([drone_id] if drone_id is not None else [])
    drone = build_drone(ip, drone_id)
    takeoff_hover_land(
        drone,
        label=f"drone-{drone_id if drone_id is not None else 'auto'}",
        ip=ip,
        height_cm=height_cm,
        hover_seconds=hover_seconds,
        connect_timeout=connect_timeout,
    )
    print("=== Single-drone demo complete ===")


def demo_two(
    ips: list[str],
    ids: list[int],
    height_cm: int,
    hover_seconds: float,
    connect_timeout: float,
) -> None:
    print("=== Two-drone demo (concurrent) ===")
    _warn_if_zero_id(ids)
    drones = [build_drone(ip, did) for ip, did in zip(ips, ids)]

    errors: dict[str, BaseException] = {}

    def worker(drone: DroneAPI, ip: str, did: int) -> None:
        label = f"drone-{did}"
        try:
            takeoff_hover_land(
                drone,
                label=label,
                ip=ip,
                height_cm=height_cm,
                hover_seconds=hover_seconds,
                connect_timeout=connect_timeout,
            )
        except BaseException as exc:  # noqa: BLE001 - surface any failure per-thread
            errors[label] = exc
            print(f"[{label}] ERROR: {exc}", flush=True)

    threads = [
        threading.Thread(target=worker, args=(d, ip, did), name=f"drone-{did}")
        for d, ip, did in zip(drones, ips, ids)
    ]

    # Both drones take off and hover at the same time; identity is kept distinct
    # by the explicit per-connection drone_id, not by staggering.
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("=== Two-drone demo complete ===")
    if errors:
        raise SystemExit(f"{len(errors)} drone(s) reported errors: {list(errors)}")


def check_wiring(ips: list[str], ids: list[int]) -> None:
    """Construct the API objects without any network I/O and verify identity.

    Useful as a smoke test on a machine with no drones attached.
    """
    print("=== Wiring check (no hardware) ===")
    print(f"pyhulax loaded from: {os.path.dirname(pyhulax.__file__)}")
    for ip, did in zip(ips, ids):
        drone = build_drone(ip, did)
        server = drone._server  # noqa: SLF001 - demo introspection only
        resolved = server._resolve_command_id()  # noqa: SLF001
        assert resolved == did, f"expected command id {did}, got {resolved}"
        assert server._drone_id == did  # noqa: SLF001
        assert drone.default_ip == ip, f"expected ip {ip}, got {drone.default_ip}"
        print(f"  drone_id={did:<3} ip={ip:<15} -> command plane_id resolves to {resolved}")
    print("=== Wiring check passed ===")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["one", "two"],
                        help="Run the one-drone or two-drone demo")
    parser.add_argument("--ips", nargs="+", default=None,
                        help="Drone IP(s). Defaults: one=192.168.100.1, "
                             "two=192.168.100.1 192.168.100.2")
    parser.add_argument("--ids", nargs="+", type=int, default=None,
                        help="Explicit drone id(s). Defaults: one=1, two=1 2")
    parser.add_argument("--height", type=int, default=100,
                        help="Takeoff/hover height in cm (default 100)")
    parser.add_argument("--hover", type=float, default=5.0,
                        help="Hover duration in seconds (default 5)")
    parser.add_argument("--connect-timeout", type=float, default=8.0,
                        help="Seconds to wait for each drone's heartbeat (default 8)")
    parser.add_argument("--check", action="store_true",
                        help="Validate wiring only; do not connect or fly")
    args = parser.parse_args(argv)

    if args.mode == "one":
        ips = args.ips or ["192.168.100.1"]
        ids = args.ids or [1]
    else:
        ips = args.ips or ["192.168.100.1", "192.168.100.2"]
        ids = args.ids or [1, 2]

    expected = 1 if args.mode == "one" else 2
    if len(ips) != expected or len(ids) != expected:
        parser.error(f"mode '{args.mode}' needs {expected} ip(s) and {expected} id(s); "
                     f"got {len(ips)} ip(s) and {len(ids)} id(s)")

    if args.check:
        check_wiring(ips, ids)
        return

    if args.mode == "one":
        demo_one(ips[0], ids[0], args.height, args.hover, args.connect_timeout)
    else:
        demo_two(ips, ids, args.height, args.hover, args.connect_timeout)


if __name__ == "__main__":
    main()
