"""Data models for the manual flight controller."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from pyhulax.config import DroneConfig, get_config

if TYPE_CHECKING:
    from pyhulax.system.datacenter import DataCenter

_cfg = get_config()


@dataclass
class FlightState:
    """Represents drone position and orientation state.

    Attributes:
        x: X position in cm (East on QR mat)
        y: Y position in cm (North on QR mat)
        z: Z position/altitude in cm
        yaw: Heading in degrees (0-360, CCW positive)
        timestamp: Monotonic time for derivative calculation
    """

    x: float
    y: float
    z: float
    yaw: float
    timestamp: float = field(default_factory=time.monotonic)

    @classmethod
    def from_telemetry(cls, datacenter: DataCenter) -> Optional[FlightState]:
        """Create FlightState from DataCenter telemetry.

        Args:
            datacenter: DataCenter instance with drone telemetry

        Returns:
            FlightState if telemetry available, None otherwise
        """
        flight_data = datacenter.get_data("Plane", "flight_data")
        if flight_data is None:
            return None

        return cls(
            x=float(flight_data.x),
            y=float(flight_data.y),
            z=float(flight_data.z),
            yaw=flight_data.yaw / 100.0,  # 0.01 deg units -> degrees
            timestamp=time.monotonic(),
        )

    def distance_to(self, other: FlightState) -> float:
        """Compute 3D Euclidean distance to another state.

        Args:
            other: Target FlightState

        Returns:
            Distance in cm
        """
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )

    def yaw_error_to(self, target_yaw: float) -> float:
        """Compute yaw error with wraparound handling.

        Computes the shortest angular distance from current yaw to target.

        Args:
            target_yaw: Target heading in degrees

        Returns:
            Error in degrees (-180 to +180). Positive = CCW rotation needed.
        """
        error = target_yaw - self.yaw
        # Normalize to [-180, 180] for shortest path
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    def __repr__(self) -> str:
        return f"FlightState(x={self.x:.1f}, y={self.y:.1f}, z={self.z:.1f}, yaw={self.yaw:.1f})"


@dataclass
class ControllerConfig:
    """Configuration for the PD flight controller.

    Attributes:
        kp_xy: Proportional gain for horizontal (X, Y) control
        kd_xy: Derivative gain for horizontal control
        kp_z: Proportional gain for altitude (Z) control
        kd_z: Derivative gain for altitude control
        kp_yaw: Proportional gain for yaw control
        kd_yaw: Derivative gain for yaw control
        max_horizontal_output: Max output for X/Y axes (0-1000)
        max_vertical_output: Max output for Z axis (0-1000)
        max_yaw_output: Max output for yaw axis (0-1000)
        position_tolerance_cm: Position convergence tolerance
        yaw_tolerance_deg: Yaw convergence tolerance
        control_rate_hz: Control loop frequency
        timeout_sec: Maximum time for a fly_to operation
        min_altitude_cm: Minimum allowed altitude (safety)
        max_altitude_cm: Maximum allowed altitude (safety)
    """

    # Position PD gains (from central config)
    kp_xy: float = _cfg.controller.kp_xy
    kd_xy: float = _cfg.controller.kd_xy
    kp_z: float = _cfg.controller.kp_z
    kd_z: float = _cfg.controller.kd_z

    # Yaw PD gains
    kp_yaw: float = _cfg.controller.kp_yaw
    kd_yaw: float = _cfg.controller.kd_yaw

    # Output limits (maps to -1000 to +1000 manual control range)
    max_horizontal_output: float = _cfg.controller.max_horizontal_output
    max_vertical_output: float = _cfg.controller.max_vertical_output
    max_yaw_output: float = _cfg.controller.max_yaw_output

    # Convergence tolerances
    position_tolerance_cm: float = _cfg.flight.position_tolerance_cm
    yaw_tolerance_deg: float = _cfg.flight.yaw_tolerance_deg

    # Control loop parameters
    control_rate_hz: float = _cfg.controller.control_rate_hz
    timeout_sec: float = _cfg.timeouts.fly_to_timeout_sec

    # Safety parameters
    min_altitude_cm: float = _cfg.physics.min_altitude_cm
    max_altitude_cm: float = _cfg.physics.max_altitude_cm

    @property
    def dt(self) -> float:
        """Control loop period in seconds."""
        return 1.0 / self.control_rate_hz

    @classmethod
    def from_drone_config(cls, config: DroneConfig) -> ControllerConfig:
        """Create controller defaults from a runtime drone config."""
        return cls(
            kp_xy=config.controller.kp_xy,
            kd_xy=config.controller.kd_xy,
            kp_z=config.controller.kp_z,
            kd_z=config.controller.kd_z,
            kp_yaw=config.controller.kp_yaw,
            kd_yaw=config.controller.kd_yaw,
            max_horizontal_output=config.controller.max_horizontal_output,
            max_vertical_output=config.controller.max_vertical_output,
            max_yaw_output=config.controller.max_yaw_output,
            position_tolerance_cm=config.flight.position_tolerance_cm,
            yaw_tolerance_deg=config.flight.yaw_tolerance_deg,
            control_rate_hz=config.controller.control_rate_hz,
            timeout_sec=config.timeouts.fly_to_timeout_sec,
            min_altitude_cm=config.physics.min_altitude_cm,
            max_altitude_cm=config.physics.max_altitude_cm,
        )


@dataclass
class ControllerResult:
    """Result of a fly_to operation.

    Attributes:
        success: True if target was reached within tolerance
        final_state: Drone state at completion
        error_position_cm: Final position error in cm
        error_yaw_deg: Final yaw error in degrees
        elapsed_sec: Time taken for the operation
        reason: Human-readable result description
    """

    success: bool
    final_state: Optional[FlightState] = None
    error_position_cm: float = 0.0
    error_yaw_deg: float = 0.0
    elapsed_sec: float = 0.0
    reason: str = ""

    def __repr__(self) -> str:
        status = "SUCCESS" if self.success else "FAILED"
        return (
            f"ControllerResult({status}, "
            f"pos_err={self.error_position_cm:.1f}cm, "
            f"yaw_err={self.error_yaw_deg:.1f}deg, "
            f"elapsed={self.elapsed_sec:.2f}s, "
            f"reason='{self.reason}')"
        )
