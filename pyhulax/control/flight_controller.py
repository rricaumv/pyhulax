"""Closed-loop PD flight controller using manual control inputs."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Optional

from pyhulax.control.models import ControllerConfig, ControllerResult, FlightState
from pyhulax.core.models import Obstacles
from pyhulax.core.types import CommandResult, VelocityLevel

if TYPE_CHECKING:
    from pyhulax import DroneAPI


class ManualFlightController:
    """Closed-loop PD controller using manual control inputs.

    This controller enables simultaneous position (XYZ) and yaw control,
    which is not possible with the blocking move_to() + rotate() commands.

    The control loop runs at a configurable rate (default 20Hz) and uses
    PD control to smoothly reach the target state.

    Coordinate Frame (World-Frame):

    -  +X = East (right on QR mat)
    -  +Y = North (forward on QR mat)
    -  +Z = Up
    -  +yaw = CCW rotation

    Example:
    ```python
    from pyhulax import DroneAPI

    with DroneAPI() as drone:
        drone.connect("192.168.100.1")
        drone.takeoff()

        # Create controller
        ctrl = drone.create_flight_controller()
        ctrl.configure(kp_xy=2.5, position_tolerance_cm=3.0)

        # Fly to target with simultaneous yaw
        result = ctrl.fly_to(x=100, y=200, z=120, yaw=90)
        print(f"Arrived: {result.success}, error: {result.error_position_cm}cm")

        drone.land()
    ```
    """

    def __init__(self, drone: DroneAPI, config: Optional[ControllerConfig] = None):
        """Initialize controller with DroneAPI instance.

        Args:
            drone: Connected DroneAPI instance.
            config: Controller configuration. Uses defaults if None.
        """
        self._drone = drone
        self._config = config or ControllerConfig()
        self._target: Optional[FlightState] = None
        self._prev_state: Optional[FlightState] = None
        self._prev_error: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def configure(self, **kwargs: Any) -> ManualFlightController:
        """Fluent configuration of controller parameters.

        Args:
            **kwargs: Any ControllerConfig attribute to override.
                Common: kp_xy, kd_xy, kp_z, kd_z, kp_yaw, kd_yaw,
                       position_tolerance_cm, yaw_tolerance_deg

        Returns:
            Self for method chaining.

        Example:
        ```python
        ctrl.configure(kp_xy=2.5, position_tolerance_cm=3.0)
        ```
        """
        for key, value in kwargs.items():
            if hasattr(self._config, key):
                setattr(self._config, key, value)
            else:
                raise ValueError(f"Unknown config parameter: {key}")
        return self

    def set_target(
        self, x: float, y: float, z: float, yaw: Optional[float] = None
    ) -> None:
        """Set target state for the controller.

        Args:
            x: Target X position in cm (East on QR mat).
            y: Target Y position in cm (North on QR mat).
            z: Target altitude in cm.
            yaw: Target heading in degrees (0-360). None = maintain current heading.
        """
        # Clamp altitude to safety limits
        z = max(self._config.min_altitude_cm, min(self._config.max_altitude_cm, z))

        # If yaw not specified, use current heading
        if yaw is None:
            current = self._get_current_state()
            yaw = current.yaw if current else 0.0

        # Normalize yaw to [0, 360)
        if yaw is not None:
            yaw = yaw % 360

        self._target = FlightState(x=x, y=y, z=z, yaw=yaw)
        self._prev_error = (0.0, 0.0, 0.0, 0.0)

    def _get_current_state(self) -> Optional[FlightState]:
        """Get current drone state from telemetry.

        Returns:
            FlightState if telemetry available, None otherwise.
        """
        datacenter = self._drone._server._taskcontroller._datacenter
        return FlightState.from_telemetry(datacenter)

    def update(self) -> bool:
        """Execute one control iteration.

        Computes PD control outputs and sends manual control command.

        Returns:
            True if update succeeded, False on telemetry error.
        """
        if self._target is None:
            return False

        # Get current state
        current = self._get_current_state()
        if current is None:
            return False

        # Compute position errors (target - current)
        error_x = self._target.x - current.x
        error_y = self._target.y - current.y
        error_z = self._target.z - current.z
        error_yaw = current.yaw_error_to(self._target.yaw)

        # Compute derivatives (change in error / dt)
        dt = self._config.dt
        if self._prev_state is not None:
            actual_dt = current.timestamp - self._prev_state.timestamp
            if actual_dt > 0:
                dt = actual_dt

        d_error_x = (error_x - self._prev_error[0]) / dt
        d_error_y = (error_y - self._prev_error[1]) / dt
        d_error_z = (error_z - self._prev_error[2]) / dt
        d_error_yaw = (error_yaw - self._prev_error[3]) / dt

        # PD control: u = Kp * e + Kd * de/dt
        cfg = self._config

        # Horizontal control (world-frame)
        output_x = cfg.kp_xy * error_x + cfg.kd_xy * d_error_x
        output_y = cfg.kp_xy * error_y + cfg.kd_xy * d_error_y

        # Vertical control
        output_z = cfg.kp_z * error_z + cfg.kd_z * d_error_z

        # Yaw control
        output_yaw = cfg.kp_yaw * error_yaw + cfg.kd_yaw * d_error_yaw

        # Clamp outputs to limits
        output_x = max(-cfg.max_horizontal_output, min(cfg.max_horizontal_output, output_x))
        output_y = max(-cfg.max_horizontal_output, min(cfg.max_horizontal_output, output_y))
        output_z = max(-cfg.max_vertical_output, min(cfg.max_vertical_output, output_z))
        output_yaw = max(-cfg.max_yaw_output, min(cfg.max_yaw_output, output_yaw))

        # Send manual control (world-frame coordinates)
        # Manual control: x=forward(+Y), y=right(+X), z=up(+Z), r=CW rotation
        # Our world frame: +X=East, +Y=North, +Z=Up, +yaw=CCW
        # Mapping: forward=output_y, right=output_x, up=output_z, rotate=-output_yaw
        self._drone.send_manual_control(
            forward=output_y / 1000.0,  # +Y (North) maps to forward
            right=output_x / 1000.0,    # +X (East) maps to right
            up=output_z / 1000.0,       # +Z (Up) maps to up
            rotate=-output_yaw / 1000.0  # +yaw (CCW) maps to -rotate (CW positive in manual control)
        )

        # Store for next iteration
        self._prev_state = current
        self._prev_error = (error_x, error_y, error_z, error_yaw)

        return True

    def has_converged(self) -> bool:
        """Check if drone has reached target within tolerance.

        Returns:
            True if position and yaw errors are within configured tolerances.
        """
        if self._target is None:
            return False

        current = self._get_current_state()
        if current is None:
            return False

        # Check position error
        pos_error = current.distance_to(self._target)
        if pos_error > self._config.position_tolerance_cm:
            return False

        # Check yaw error
        yaw_error = abs(current.yaw_error_to(self._target.yaw))
        if yaw_error > self._config.yaw_tolerance_deg:
            return False

        return True

    def stop(self) -> None:
        """Send zero control inputs to stop all movement."""
        self._drone.send_manual_control(forward=0.0, right=0.0, up=0.0, rotate=0.0)
        self._target = None

    def fly_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> ControllerResult:
        """Blocking fly to target position and heading.

        Runs the control loop until convergence or timeout.

        Args:
            x: Target X position in cm.
            y: Target Y position in cm.
            z: Target altitude in cm.
            yaw: Target heading in degrees. None = maintain current heading.
            timeout: Maximum time in seconds. Uses config default if None.

        Returns:
            ControllerResult with success status and final state.
        """
        timeout = timeout or self._config.timeout_sec
        start_time = time.monotonic()

        # Set target
        self.set_target(x, y, z, yaw)

        # Control loop
        loop_period = self._config.dt
        while True:
            loop_start = time.monotonic()
            elapsed = loop_start - start_time

            # Check timeout
            if elapsed >= timeout:
                final_state = self._get_current_state()
                target = self._target  # Captured for type narrowing
                pos_error = final_state.distance_to(target) if final_state and target else 0.0
                yaw_error = abs(final_state.yaw_error_to(target.yaw)) if final_state and target else 0.0
                self.stop()
                return ControllerResult(
                    success=False,
                    final_state=final_state,
                    error_position_cm=pos_error,
                    error_yaw_deg=yaw_error,
                    elapsed_sec=elapsed,
                    reason="Timeout",
                )

            # Check convergence
            if self.has_converged():
                final_state = self._get_current_state()
                target = self._target  # Captured for type narrowing
                pos_error = final_state.distance_to(target) if final_state and target else 0.0
                yaw_error = abs(final_state.yaw_error_to(target.yaw)) if final_state and target else 0.0
                self.stop()
                return ControllerResult(
                    success=True,
                    final_state=final_state,
                    error_position_cm=pos_error,
                    error_yaw_deg=yaw_error,
                    elapsed_sec=elapsed,
                    reason="Converged",
                )

            # Execute control update
            if not self.update():
                final_state = self._get_current_state()
                self.stop()
                return ControllerResult(
                    success=False,
                    final_state=final_state,
                    elapsed_sec=elapsed,
                    reason="Telemetry unavailable",
                )

            # Sleep for remainder of loop period
            loop_elapsed = time.monotonic() - loop_start
            sleep_time = loop_period - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # --- DroneInterface compatibility methods ---

    def move_to(
        self, x: float, y: float, z: float, speed: VelocityLevel | int = VelocityLevel.ZOOM
    ) -> bool:
        """Move to absolute position (cm).

        DroneInterface-compatible method. Speed parameter affects P-gain scaling.

        Args:
            x: X position in cm.
            y: Y position in cm.
            z: Z position (altitude) in cm.
            speed: Speed level (affects gains). Higher = more aggressive.

        Returns:
            True if movement succeeded.
        """
        # Adjust gains based on speed
        # VelocityLevel: TURBO=50, ZOOM=100, MEDIUM=200, SLOW=300
        # Lower value = faster/more aggressive
        speed_value = speed.value if isinstance(speed, VelocityLevel) else speed
        if speed_value == VelocityLevel.TURBO.value:
            gain_scale = 1.5
        elif speed_value == VelocityLevel.ZOOM.value:
            gain_scale = 1.0
        elif speed_value == VelocityLevel.MEDIUM.value:
            gain_scale = 0.7
        else:
            gain_scale = 0.5

        # Temporarily adjust gains
        orig_kp = self._config.kp_xy
        self._config.kp_xy = orig_kp * gain_scale

        try:
            result = self.fly_to(x, y, z, yaw=None)
            return result.success
        finally:
            self._config.kp_xy = orig_kp

    def get_position(self) -> tuple[float, float, float]:
        """Get current (x, y, z) position in cm.

        Returns:
            Tuple of (x, y, z) in cm.
        """
        state = self._get_current_state()
        if state is None:
            return (0.0, 0.0, 0.0)
        return (state.x, state.y, state.z)

    def get_yaw(self) -> float:
        """Get current heading in degrees.

        Returns:
            Yaw angle in degrees (0-360).
        """
        state = self._get_current_state()
        if state is None:
            return 0.0
        return state.yaw

    def get_obstacles(self) -> Obstacles:
        """Get obstacle readings from barrier sensors.

        Returns:
            Obstacles with forward, back, left, right, down flags.
        """
        return self._drone.get_obstacles()

    def rotate(self, degrees: float) -> bool:
        """Rotate by degrees using the flight controller.

        Args:
            degrees: Rotation angle. Positive = CCW, negative = CW.

        Returns:
            True if rotation succeeded.
        """
        current = self._get_current_state()
        if current is None:
            return False

        target_yaw = (current.yaw + degrees) % 360
        result = self.fly_to(current.x, current.y, current.z, yaw=target_yaw)
        return result.success

    def takeoff(self, height_cm: int = 100) -> bool:
        """Take off to specified height.

        Delegates to underlying DroneAPI.

        Args:
            height_cm: Target altitude in cm.

        Returns:
            True if takeoff succeeded.
        """
        result = self._drone.takeoff(height_cm=height_cm)
        return result == CommandResult.SUCCESS

    def land(self) -> bool:
        """Land the drone.

        Delegates to underlying DroneAPI.

        Returns:
            True if landing succeeded.
        """
        result = self._drone.land()
        return result == CommandResult.SUCCESS

    def hover(self, duration_sec: float = 1.0) -> bool:
        """Hover in place using the controller.

        Args:
            duration_sec: Hover duration in seconds.

        Returns:
            True if hover completed.
        """
        current = self._get_current_state()
        if current is None:
            return False

        # Set target to current position and run control loop
        self.set_target(current.x, current.y, current.z, current.yaw)

        start_time = time.monotonic()
        loop_period = self._config.dt

        while time.monotonic() - start_time < duration_sec:
            loop_start = time.monotonic()
            if not self.update():
                self.stop()
                return False

            loop_elapsed = time.monotonic() - loop_start
            sleep_time = loop_period - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.stop()
        return True

    @property
    def drone(self) -> DroneAPI:
        """Get underlying DroneAPI instance for direct access if needed."""
        return self._drone

    @property
    def config(self) -> ControllerConfig:
        """Get current controller configuration."""
        return self._config
