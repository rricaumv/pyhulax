"""
Custom exceptions for Drone API.

Replaces silent failures (returning 0, [0,0,0], etc.) with proper exceptions.
"""


class DroneError(Exception):
    """Base exception for all drone API errors."""
    pass


class DroneConnectionError(DroneError):
    """Failed to connect to drone.

    Raised when:

    - Network connection to drone IP fails
    - TCP/UDP socket creation fails
    - Initial handshake times out
    """
    def __init__(self, message: str = "Failed to connect to drone", ip: str | None = None):
        self.ip = ip
        super().__init__(f"{message}" + (f" at {ip}" if ip else ""))


class CommandTimeout(DroneError):
    """Command did not receive acknowledgment in time.

    Raised when:

    - No PLANE_ACK received within timeout
    - Task processor wait_state times out
    """
    def __init__(self, command: str, timeout_seconds: float):
        self.command = command
        self.timeout = timeout_seconds
        super().__init__(f"Command '{command}' timed out after {timeout_seconds}s")


class CommandRejected(DroneError):
    """Drone rejected the command.

    Raised when:

    - PLANE_ACK returns non-success result code
    - Command validation fails on drone
    """
    def __init__(self, command: str, result_code: int, message: str = ""):
        self.command = command
        self.result_code = result_code
        detail = f": {message}" if message else ""
        super().__init__(f"Command '{command}' rejected with code {result_code}{detail}")


class NotReady(DroneError):
    """Drone is not in ready state for this operation.

    Raised when:

    - Attempting to fly without connection
    - Attempting takeoff while already flying
    - No heartbeat received from drone
    """
    def __init__(self, message: str = "Drone not ready for operation"):
        super().__init__(message)


class LowBattery(DroneError):
    """Battery level too low for operation.

    Raised when:

    - Battery below threshold for requested operation
    - Emergency low battery state detected
    """
    def __init__(self, battery_percent: int, threshold: int = 10):
        self.battery_percent = battery_percent
        self.threshold = threshold
        super().__init__(f"Battery at {battery_percent}% (threshold: {threshold}%)")


class TelemetryUnavailable(DroneError):
    """Requested telemetry data not available.

    Raised when:

    - DataCenter has no flight_data for drone
    - Telemetry update not received yet
    - Specific sensor data missing
    """
    def __init__(self, data_type: str = "telemetry"):
        self.data_type = data_type
        super().__init__(f"{data_type} data not available")


class InvalidParameter(DroneError):
    """Invalid parameter value for command.

    Raised when:

    - Parameter out of valid range
    - Invalid enum value
    - Incompatible parameter combination
    """
    def __init__(self, param_name: str, value, valid_range: str = ""):
        self.param_name = param_name
        self.value = value
        range_info = f" (valid: {valid_range})" if valid_range else ""
        super().__init__(f"Invalid {param_name}={value}{range_info}")


class OperationInProgress(DroneError):
    """Another operation is already in progress.

    Raised when:

    - Attempting to start new command while previous running
    - Task queue is full
    """
    def __init__(self, operation: str = "operation"):
        self.operation = operation
        super().__init__(f"Cannot start new command: {operation} in progress")
