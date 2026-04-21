"""
Flight logging module for Drone API.

Provides database logging for flight sessions, telemetry, and commands.
Supports both SQLite (sync) and PostgreSQL (async) backends.

Example usage:
```python
# SQLite (sync)
from pyhulax.logging import SQLiteLogger
logger = SQLiteLogger("flights.db")
session_id = logger.start_session(drone_id=1, notes="Test flight")
logger.log_telemetry(session_id, flight_data)
logger.end_session(session_id)
```

```python
# PostgreSQL (async)
from pyhulax.logging import PostgresLogger
logger = PostgresLogger("postgresql://user:pass@localhost/drone")
await logger.connect()
session_id = await logger.start_session(drone_id=1)
await logger.log_telemetry(session_id, flight_data)
await logger.end_session(session_id)
```
"""

from .base import FlightLogger
from .models import FlightSession, TelemetryRecord, CommandRecord
from .sqlite_logger import SQLiteLogger
from .file_logger import FileLoggerMiddleware
from .command_logger import CommandLogger, create_logging_wrapper

# PostgresLogger imported conditionally to avoid asyncpg dependency
try:
    from .postgres_logger import PostgresLogger
except ImportError:
    PostgresLogger = None  # type: ignore

__all__ = [
    "FlightLogger",
    "FlightSession",
    "TelemetryRecord",
    "CommandRecord",
    "SQLiteLogger",
    "PostgresLogger",
    "FileLoggerMiddleware",
    "CommandLogger",
    "create_logging_wrapper",
]
