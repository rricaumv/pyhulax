"""
Command logging for DroneAPI.

Logs every API command with arguments and results to JSONL files.
Similar to FileLoggerMiddleware but for outgoing commands instead of incoming telemetry.
"""

import functools
import inspect
import json
import sys
from datetime import datetime, date
from enum import Enum
from pathlib import Path
from threading import Lock
from typing import Any, Callable, Optional, TextIO, TypeVar, ParamSpec

P = ParamSpec("P")
R = TypeVar("R")


class CommandLogger:
    """
    Thread-safe logger for DroneAPI commands.

    Logs every API method call with:

    - Timestamp
    - Method name
    - Arguments (positional and keyword)
    - Return value or exception
    - Execution time

    Files are organized as: `logs/commands_YYYY-MM-DD.jsonl`

    Usage:
    ```python
    logger = CommandLogger("logs")

    @logger.log
    def move(self, direction: Direction, distance: int) -> CommandResult:
        ...

    # Or wrap an existing instance:
    drone = DroneAPI()
    logged_drone = logger.wrap(drone)
    ```
    """

    def __init__(
        self,
        log_dir: str = "logs",
        prefix: str = "commands",
        log_args: bool = True,
        log_result: bool = True,
        log_exceptions: bool = True,
        exclude_methods: set[str] | None = None,
    ):
        """
        Initialize the command logger.

        Args:
            log_dir: Directory to store log files
            prefix: Filename prefix (default: "commands")
            log_args: Whether to log method arguments
            log_result: Whether to log return values
            log_exceptions: Whether to log exceptions
            exclude_methods: Set of method names to exclude from logging
        """
        self.log_dir = Path(log_dir)
        self.prefix = prefix
        self.log_args = log_args
        self.log_result = log_result
        self.log_exceptions = log_exceptions
        self.exclude_methods = exclude_methods or {
            # Exclude high-frequency telemetry getters by default
            "get_position",
            "get_orientation",
            "get_battery",
            "get_obstacles",
            "get_state",
            "get_flight_data",
        }

        self._lock = Lock()
        self._current_date: Optional[date] = None
        self._file_handle: Optional[TextIO] = None
        self._enabled = True

    def enable(self) -> None:
        """Enable command logging."""
        self._enabled = True

    def disable(self) -> None:
        """Disable command logging."""
        self._enabled = False

    def log(self, func: Callable[P, R]) -> Callable[P, R]:
        """
        Decorator to log a function/method call.

        Usage:
        ```python
        @logger.log
        def move(self, direction: Direction, distance: int):
            ...
        ```
        """
        @functools.wraps(func)
        def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
            if not self._enabled:
                return func(*args, **kwargs)

            method_name = func.__name__

            # Skip excluded methods
            if method_name in self.exclude_methods:
                return func(*args, **kwargs)

            # Build record
            start_time = datetime.now()
            record: dict[str, Any] = {
                "timestamp": start_time.isoformat(),
                "method": method_name,
            }

            # Log arguments (skip 'self')
            if self.log_args:
                record["args"] = self._serialize_args(func, args, kwargs)

            try:
                result = func(*args, **kwargs)

                # Log result
                if self.log_result:
                    record["result"] = self._serialize_value(result)
                record["success"] = True

                return result

            except Exception as e:
                if self.log_exceptions:
                    record["exception"] = {
                        "type": type(e).__name__,
                        "message": str(e),
                    }
                record["success"] = False
                raise

            finally:
                # Log execution time
                elapsed = (datetime.now() - start_time).total_seconds()
                record["elapsed_sec"] = round(elapsed, 4)
                self._write_record(record)

        return wrapper

    def log_call(
        self,
        method_name: str,
        args: dict[str, Any] | None = None,
        result: Any = None,
        success: bool = True,
        exception: Exception | None = None,
        elapsed_sec: float | None = None,
    ) -> None:
        """
        Manually log a command call.

        Useful when you can't use the decorator.
        """
        if not self._enabled:
            return

        if method_name in self.exclude_methods:
            return

        record: dict[str, Any] = {
            "timestamp": datetime.now().isoformat(),
            "method": method_name,
            "success": success,
        }

        if args and self.log_args:
            record["args"] = {k: self._serialize_value(v) for k, v in args.items()}

        if result is not None and self.log_result:
            record["result"] = self._serialize_value(result)

        if exception is not None and self.log_exceptions:
            record["exception"] = {
                "type": type(exception).__name__,
                "message": str(exception),
            }

        if elapsed_sec is not None:
            record["elapsed_sec"] = round(elapsed_sec, 4)

        self._write_record(record)

    def _serialize_args(
        self,
        func: Callable,
        args: tuple,
        kwargs: dict,
    ) -> dict[str, Any]:
        """Serialize function arguments, skipping 'self'."""
        result = {}

        try:
            sig = inspect.signature(func)
            params = list(sig.parameters.keys())

            # Map positional args to parameter names
            for i, (param_name, arg_value) in enumerate(zip(params, args)):
                # Skip 'self' parameter
                if param_name == "self":
                    continue
                result[param_name] = self._serialize_value(arg_value)

            # Add keyword arguments
            for key, value in kwargs.items():
                result[key] = self._serialize_value(value)

        except (ValueError, TypeError):
            # Fallback: just serialize what we can
            result["_args"] = [self._serialize_value(a) for a in args[1:]]  # Skip self
            result["_kwargs"] = {k: self._serialize_value(v) for k, v in kwargs.items()}

        return result

    def _serialize_value(self, value: Any) -> Any:
        """Convert a value to JSON-serializable type."""
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, Enum):
            return {"_enum": type(value).__name__, "value": value.name}
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, bytes):
            try:
                return value.decode("utf-8").rstrip("\x00")
            except UnicodeDecodeError:
                return f"<bytes:{len(value)}>"
        if isinstance(value, (list, tuple)):
            return [self._serialize_value(v) for v in value]
        if isinstance(value, dict):
            return {str(k): self._serialize_value(v) for k, v in value.items()}
        if hasattr(value, "__dict__"):
            # Dataclass or object with attributes
            return {
                "_type": type(value).__name__,
                **{k: self._serialize_value(v) for k, v in vars(value).items() if not k.startswith("_")},
            }
        if hasattr(value, "model_dump"):
            # Pydantic model
            return {"_type": type(value).__name__, **value.model_dump()}
        # Fallback
        return str(value)

    def _write_record(self, record: dict) -> None:
        """Thread-safe write of a record to the log file."""
        with self._lock:
            today = date.today()

            if self._current_date != today:
                self._rotate_file(today)

            if self._file_handle is not None:
                try:
                    json.dump(record, self._file_handle, separators=(",", ":"))
                    self._file_handle.write("\n")
                    self._file_handle.flush()
                except Exception as e:
                    print(f"[CommandLogger] Write error: {e}", file=sys.stderr)

    def _rotate_file(self, new_date: date) -> None:
        """Close current file and open a new one for the given date."""
        if self._file_handle is not None:
            try:
                self._file_handle.close()
            except Exception:
                pass
            self._file_handle = None

        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            print(f"[CommandLogger] Cannot create log directory: {e}", file=sys.stderr)
            return

        log_path = self.log_dir / f"{self.prefix}_{new_date.isoformat()}.jsonl"
        try:
            self._file_handle = open(log_path, "a", encoding="utf-8")
            self._current_date = new_date
            print(f"[CommandLogger] Logging to {log_path}")
        except Exception as e:
            print(f"[CommandLogger] Cannot open log file: {e}", file=sys.stderr)

    def close(self) -> None:
        """Close the log file."""
        with self._lock:
            if self._file_handle is not None:
                try:
                    self._file_handle.close()
                except Exception:
                    pass
                self._file_handle = None
                self._current_date = None


def create_logging_wrapper(obj: Any, logger: CommandLogger) -> Any:
    """
    Create a wrapper that logs all method calls on an object.

    This wraps ALL public methods (not starting with _) with logging.

    For manual_fly(), automatically injects an on_frame callback to log
    each individual MANUAL_CONTROL frame sent.

    Usage:
    ```python
    drone = DroneAPI()
    logged_drone = create_logging_wrapper(drone, CommandLogger("logs"))
    logged_drone.takeoff()  # This call is logged
    logged_drone.manual_fly(2.0, forward=0.5)  # Logs each frame too
    ```
    """

    class LoggingWrapper:
        def __init__(self, wrapped: Any, cmd_logger: CommandLogger):
            self._wrapped = wrapped
            self._logger = cmd_logger

        def __getattr__(self, name: str) -> Any:
            attr = getattr(self._wrapped, name)

            # Only wrap public callable methods
            if name.startswith("_") or not callable(attr):
                return attr

            # Special handling for manual_fly to log individual frames
            if name == "manual_fly":
                @functools.wraps(attr)
                def logged_manual_fly(*args, **kwargs):
                    # Create frame logging callback
                    def frame_logger(x, y, z, r, frame_idx, success):
                        self._logger.log_call(
                            method_name="manual_control_frame",
                            args={"x": x, "y": y, "z": z, "r": r, "frame": frame_idx},
                            result=success,
                            success=success,
                        )

                    # Inject our frame logger, preserving any user-provided one
                    user_callback = kwargs.get("on_frame")
                    if user_callback:
                        def combined_callback(x, y, z, r, idx, ok):
                            frame_logger(x, y, z, r, idx, ok)
                            user_callback(x, y, z, r, idx, ok)
                        kwargs["on_frame"] = combined_callback
                    else:
                        kwargs["on_frame"] = frame_logger

                    return self._logger.log(attr)(*args, **kwargs)

                return logged_manual_fly

            # Wrap with logging
            @functools.wraps(attr)
            def logged_method(*args, **kwargs):
                return self._logger.log(attr)(*args, **kwargs)

            return logged_method

    return LoggingWrapper(obj, logger)
