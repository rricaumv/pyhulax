"""
File-based logging middleware for drone status and telemetry.

Saves all parsed MAVLink messages to JSON Lines files with daily rotation.
"""

import json
import sys
from datetime import datetime, date
from pathlib import Path
from threading import Lock
from typing import Any, Optional, TextIO


class FileLoggerMiddleware:
    """
    Thread-safe file logger that writes parsed MAVLink messages to JSONL files.

    Files are organized as: `logs/drone_YYYY-MM-DD.jsonl`
    Automatically rotates to a new file at midnight.

    Usage:
    ```python
    logger = FileLoggerMiddleware("logs")
    logger.log_message(msg)  # Called from message analyzer thread
    ```

    """

    def __init__(self, log_dir: str = "logs"):
        """
        Initialize the file logger.

        Args:
            log_dir: Directory to store log files (created if missing)
        """
        self.log_dir = Path(log_dir)
        self._lock = Lock()
        self._current_date: Optional[date] = None
        self._file_handle: Optional[TextIO] = None

    def log_message(
        self,
        msg: Any,
        state: Optional[Any] = None,
    ) -> None:
        """
        Log a parsed MAVLink message to the current day's log file.

        Args:
            msg: MAVLink message object with get_msg_id() method
            state: Optional State object from StateProcessor
        """
        try:
            record = {
                "timestamp": datetime.now().isoformat(),
                "msg_type": type(msg).__name__,
                "msg_id": msg.get_msg_id() if hasattr(msg, 'get_msg_id') else None,
                "data": self._serialize_mavlink_msg(msg),
            }

            if state is not None:
                record["state"] = self._serialize_state(state)

            self._write_record(record)

        except Exception as e:
            print(f"[FileLogger] Error logging message: {e}", file=sys.stderr)

    # Fields that are in centidegrees and need conversion to degrees
    CENTIDEGREE_FIELDS = {'yaw', 'pitch', 'roll'}

    def _serialize_mavlink_msg(self, msg: Any) -> dict:
        """
        Extract all fields from a MAVLink message object.

        MAVLink messages have their fields stored as attributes.
        We extract all non-private, non-callable attributes.
        Orientation fields (yaw, pitch, roll) are converted from centidegrees to degrees.
        """
        data = {}

        # Get fieldnames if available (pymavlink messages have this)
        if hasattr(msg, 'fieldnames'):
            for field in msg.fieldnames:
                value = getattr(msg, field, None)
                # Convert centidegrees to degrees for orientation fields
                if field in self.CENTIDEGREE_FIELDS and value is not None:
                    value = value / 100.0
                data[field] = self._serialize_value(value)
        else:
            # Fallback: extract all public attributes
            for attr in dir(msg):
                if attr.startswith('_'):
                    continue
                if callable(getattr(msg, attr)):
                    continue
                if attr in ('fieldnames', 'fieldtypes', 'fielddisplays_by_name',
                           'fieldenums_by_name', 'fieldunits_by_name'):
                    continue
                try:
                    value = getattr(msg, attr)
                    # Convert centidegrees to degrees for orientation fields
                    if attr in self.CENTIDEGREE_FIELDS and value is not None:
                        value = value / 100.0
                    data[attr] = self._serialize_value(value)
                except Exception:
                    pass

        return data

    def _serialize_state(self, state: Any) -> dict:
        """Serialize a State object."""
        result = {}

        if hasattr(state, 'get_state'):
            state_enum = state.get_state()
            result["type"] = state_enum.name if hasattr(state_enum, 'name') else str(state_enum)

        if hasattr(state, 'get_data'):
            data = state.get_data()
            if data is not None:
                result["data"] = {
                    k: self._serialize_value(v) for k, v in data.items()
                }

        if hasattr(state, 'get_time'):
            result["time"] = state.get_time()

        return result

    def _serialize_value(self, value: Any) -> Any:
        """Convert a value to a JSON-serializable type."""
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, bytes):
            # Try to decode as string, otherwise hex encode
            try:
                return value.decode('utf-8').rstrip('\x00')
            except UnicodeDecodeError:
                return value.hex()
        if isinstance(value, (list, tuple)):
            return [self._serialize_value(v) for v in value]
        if isinstance(value, dict):
            return {k: self._serialize_value(v) for k, v in value.items()}
        # Fallback: convert to string
        return str(value)

    def _write_record(self, record: dict) -> None:
        """Thread-safe write of a record to the log file."""
        with self._lock:
            today = date.today()

            # Rotate file if date changed
            if self._current_date != today:
                self._rotate_file(today)

            if self._file_handle is not None:
                try:
                    json.dump(record, self._file_handle, separators=(',', ':'))
                    self._file_handle.write('\n')
                    self._file_handle.flush()
                except Exception as e:
                    print(f"[FileLogger] Write error: {e}", file=sys.stderr)

    def _rotate_file(self, new_date: date) -> None:
        """Close current file and open a new one for the given date."""
        # Close existing file
        if self._file_handle is not None:
            try:
                self._file_handle.close()
            except Exception:
                pass
            self._file_handle = None

        # Create logs directory if needed
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            print(f"[FileLogger] Cannot create log directory: {e}", file=sys.stderr)
            return

        # Open new file
        log_path = self.log_dir / f"drone_{new_date.isoformat()}.jsonl"
        try:
            self._file_handle = open(log_path, 'a', encoding='utf-8')
            self._current_date = new_date
            print(f"[FileLogger] Logging to {log_path}")
        except Exception as e:
            print(f"[FileLogger] Cannot open log file: {e}", file=sys.stderr)

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
