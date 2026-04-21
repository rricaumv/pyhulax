"""
PostgreSQL flight logger implementation (async).

Asynchronous logging backend using asyncpg. Suitable for high-frequency
telemetry logging and multi-drone setups.
"""

import json
import uuid
import asyncio
from datetime import datetime
from typing import Optional, List, Any

try:
    import asyncpg
except ImportError:
    asyncpg = None  # type: ignore

from ..core.models import FlightData
from .base import FlightLogger
from .models import FlightSession, TelemetryRecord, CommandRecord


class PostgresLogger(FlightLogger):
    """
    PostgreSQL-backed async flight logger.

    Uses asyncpg for non-blocking database operations. Ideal for
    high-frequency telemetry logging.

    Usage:
    ```python
    logger = PostgresLogger("postgresql://user:pass@localhost/drone")
    await logger.connect()
    session_id = await logger.start_session_async(drone_id=1)
    await logger.log_telemetry_async(session_id, flight_data)
    await logger.close_async()
    ```
    """

    SCHEMA = """
    -- Flight sessions table
    CREATE TABLE IF NOT EXISTS sessions (
        session_id TEXT PRIMARY KEY,
        drone_id INTEGER NOT NULL,
        start_time TIMESTAMPTZ NOT NULL,
        end_time TIMESTAMPTZ,
        notes TEXT
    );

    -- Telemetry data table
    CREATE TABLE IF NOT EXISTS telemetry (
        id SERIAL PRIMARY KEY,
        session_id TEXT NOT NULL REFERENCES sessions(session_id),
        timestamp TIMESTAMPTZ NOT NULL,
        pos_x REAL NOT NULL,
        pos_y REAL NOT NULL,
        pos_z REAL NOT NULL,
        vel_x REAL NOT NULL,
        vel_y REAL NOT NULL,
        vel_z REAL NOT NULL,
        yaw REAL NOT NULL,
        pitch REAL NOT NULL,
        roll REAL NOT NULL,
        altitude REAL NOT NULL,
        battery INTEGER NOT NULL,
        barrier INTEGER DEFAULT 0
    );

    -- Commands table
    CREATE TABLE IF NOT EXISTS commands (
        id SERIAL PRIMARY KEY,
        session_id TEXT NOT NULL REFERENCES sessions(session_id),
        timestamp TIMESTAMPTZ NOT NULL,
        command TEXT NOT NULL,
        params JSONB,
        result INTEGER NOT NULL,
        duration_ms REAL NOT NULL
    );

    -- Indexes for common queries
    CREATE INDEX IF NOT EXISTS idx_telemetry_session ON telemetry(session_id);
    CREATE INDEX IF NOT EXISTS idx_telemetry_timestamp ON telemetry(timestamp);
    CREATE INDEX IF NOT EXISTS idx_commands_session ON commands(session_id);
    CREATE INDEX IF NOT EXISTS idx_sessions_drone ON sessions(drone_id);
    CREATE INDEX IF NOT EXISTS idx_sessions_start ON sessions(start_time);
    """

    def __init__(
        self,
        dsn: str,
        min_connections: int = 2,
        max_connections: int = 10,
    ):
        """
        Initialize PostgreSQL logger.

        Args:
            dsn: PostgreSQL connection string (e.g., "postgresql://user:pass@localhost/drone")
            min_connections: Minimum pool connections
            max_connections: Maximum pool connections
        """
        if asyncpg is None:
            raise ImportError("asyncpg is required for PostgresLogger. Install with: pip install asyncpg")

        self.dsn = dsn
        self.min_connections = min_connections
        self.max_connections = max_connections
        self._pool: Optional[asyncpg.Pool] = None

    async def connect(self) -> None:
        """Connect to database and initialize schema."""
        self._pool = await asyncpg.create_pool(
            self.dsn,
            min_size=self.min_connections,
            max_size=self.max_connections,
        )
        await self._init_schema()

    async def _init_schema(self) -> None:
        """Initialize database schema."""
        async with self._pool.acquire() as conn:
            await conn.execute(self.SCHEMA)

    def _ensure_connected(self) -> None:
        """Ensure pool is connected."""
        if self._pool is None:
            raise RuntimeError("Not connected. Call connect() first.")

    # ==================== Async Session Management ====================

    async def start_session_async(
        self,
        drone_id: int,
        notes: Optional[str] = None,
    ) -> str:
        """Start a new flight session (async)."""
        self._ensure_connected()
        session_id = str(uuid.uuid4())
        async with self._pool.acquire() as conn:
            await conn.execute(
                """INSERT INTO sessions (session_id, drone_id, start_time, notes)
                   VALUES ($1, $2, $3, $4)""",
                session_id, drone_id, datetime.now(), notes
            )
        return session_id

    async def end_session_async(self, session_id: str) -> None:
        """End a flight session (async)."""
        self._ensure_connected()
        async with self._pool.acquire() as conn:
            await conn.execute(
                "UPDATE sessions SET end_time = $1 WHERE session_id = $2",
                datetime.now(), session_id
            )

    async def get_session_async(self, session_id: str) -> Optional[FlightSession]:
        """Get session metadata (async)."""
        self._ensure_connected()
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM sessions WHERE session_id = $1",
                session_id
            )

        if row is None:
            return None

        return FlightSession(
            session_id=row["session_id"],
            drone_id=row["drone_id"],
            start_time=row["start_time"],
            end_time=row["end_time"],
            notes=row["notes"],
        )

    async def list_sessions_async(
        self,
        drone_id: Optional[int] = None,
        start_after: Optional[datetime] = None,
        limit: int = 100,
    ) -> List[FlightSession]:
        """List flight sessions (async)."""
        self._ensure_connected()

        query = "SELECT * FROM sessions WHERE 1=1"
        params: list[Any] = []
        param_idx = 1

        if drone_id is not None:
            query += f" AND drone_id = ${param_idx}"
            params.append(drone_id)
            param_idx += 1

        if start_after is not None:
            query += f" AND start_time > ${param_idx}"
            params.append(start_after)
            param_idx += 1

        query += f" ORDER BY start_time DESC LIMIT ${param_idx}"
        params.append(limit)

        async with self._pool.acquire() as conn:
            rows = await conn.fetch(query, *params)

        return [
            FlightSession(
                session_id=row["session_id"],
                drone_id=row["drone_id"],
                start_time=row["start_time"],
                end_time=row["end_time"],
                notes=row["notes"],
            )
            for row in rows
        ]

    # ==================== Async Telemetry Logging ====================

    async def log_telemetry_async(self, session_id: str, data: FlightData) -> None:
        """Log a telemetry data point (async)."""
        self._ensure_connected()
        async with self._pool.acquire() as conn:
            await conn.execute(
                """INSERT INTO telemetry
                   (session_id, timestamp, pos_x, pos_y, pos_z,
                    vel_x, vel_y, vel_z, yaw, pitch, roll,
                    altitude, battery, barrier)
                   VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14)""",
                session_id,
                data.timestamp,
                data.position.x,
                data.position.y,
                data.position.z,
                data.velocity.x,
                data.velocity.y,
                data.velocity.z,
                data.orientation.yaw,
                data.orientation.pitch,
                data.orientation.roll,
                data.altitude_tof,
                data.battery_percent,
                data.barrier,
            )

    async def log_telemetry_batch_async(
        self,
        session_id: str,
        data: List[FlightData],
    ) -> None:
        """Log multiple telemetry data points in batch (async)."""
        self._ensure_connected()
        async with self._pool.acquire() as conn:
            await conn.executemany(
                """INSERT INTO telemetry
                   (session_id, timestamp, pos_x, pos_y, pos_z,
                    vel_x, vel_y, vel_z, yaw, pitch, roll,
                    altitude, battery, barrier)
                   VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14)""",
                [
                    (
                        session_id,
                        d.timestamp,
                        d.position.x,
                        d.position.y,
                        d.position.z,
                        d.velocity.x,
                        d.velocity.y,
                        d.velocity.z,
                        d.orientation.yaw,
                        d.orientation.pitch,
                        d.orientation.roll,
                        d.altitude_tof,
                        d.battery_percent,
                        d.barrier,
                    )
                    for d in data
                ]
            )

    async def get_telemetry_async(
        self,
        session_id: str,
        start: Optional[datetime] = None,
        end: Optional[datetime] = None,
        limit: int = 10000,
    ) -> List[TelemetryRecord]:
        """Retrieve telemetry for a session (async)."""
        self._ensure_connected()

        query = "SELECT * FROM telemetry WHERE session_id = $1"
        params: list[Any] = [session_id]
        param_idx = 2

        if start is not None:
            query += f" AND timestamp >= ${param_idx}"
            params.append(start)
            param_idx += 1

        if end is not None:
            query += f" AND timestamp <= ${param_idx}"
            params.append(end)
            param_idx += 1

        query += f" ORDER BY timestamp ASC LIMIT ${param_idx}"
        params.append(limit)

        async with self._pool.acquire() as conn:
            rows = await conn.fetch(query, *params)

        return [
            TelemetryRecord(
                id=row["id"],
                session_id=row["session_id"],
                timestamp=row["timestamp"],
                pos_x=row["pos_x"],
                pos_y=row["pos_y"],
                pos_z=row["pos_z"],
                vel_x=row["vel_x"],
                vel_y=row["vel_y"],
                vel_z=row["vel_z"],
                yaw=row["yaw"],
                pitch=row["pitch"],
                roll=row["roll"],
                altitude=row["altitude"],
                battery=row["battery"],
                barrier=row["barrier"],
            )
            for row in rows
        ]

    # ==================== Async Command Logging ====================

    async def log_command_async(
        self,
        session_id: str,
        command: str,
        params: dict,
        result: int,
        duration_ms: float,
    ) -> None:
        """Log a command execution (async)."""
        self._ensure_connected()
        async with self._pool.acquire() as conn:
            await conn.execute(
                """INSERT INTO commands
                   (session_id, timestamp, command, params, result, duration_ms)
                   VALUES ($1, $2, $3, $4, $5, $6)""",
                session_id,
                datetime.now(),
                command,
                json.dumps(params),
                result,
                duration_ms,
            )

    async def get_commands_async(
        self,
        session_id: str,
        command_filter: Optional[str] = None,
    ) -> List[CommandRecord]:
        """Retrieve commands for a session (async)."""
        self._ensure_connected()

        query = "SELECT * FROM commands WHERE session_id = $1"
        params: list[Any] = [session_id]

        if command_filter is not None:
            query += " AND command ILIKE $2"
            params.append(f"%{command_filter}%")

        query += " ORDER BY timestamp ASC"

        async with self._pool.acquire() as conn:
            rows = await conn.fetch(query, *params)

        return [
            CommandRecord(
                id=row["id"],
                session_id=row["session_id"],
                timestamp=row["timestamp"],
                command=row["command"],
                params=row["params"] if row["params"] else {},
                result=row["result"],
                duration_ms=row["duration_ms"],
            )
            for row in rows
        ]

    # ==================== Async Cleanup ====================

    async def close_async(self) -> None:
        """Close database connection pool (async)."""
        if self._pool is not None:
            await self._pool.close()
            self._pool = None

    # ==================== Sync Wrappers (for FlightLogger interface) ====================

    @staticmethod
    def _run_async(coro):
        """Run async coroutine synchronously."""
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None

        if loop is not None:
            # Already in async context, create task
            return asyncio.ensure_future(coro)
        else:
            # No running loop, use asyncio.run
            return asyncio.run(coro)

    def start_session(self, drone_id: int, notes: Optional[str] = None) -> str:
        return self._run_async(self.start_session_async(drone_id, notes))

    def end_session(self, session_id: str) -> None:
        self._run_async(self.end_session_async(session_id))

    def get_session(self, session_id: str) -> Optional[FlightSession]:
        return self._run_async(self.get_session_async(session_id))

    def list_sessions(
        self,
        drone_id: Optional[int] = None,
        start_after: Optional[datetime] = None,
        limit: int = 100,
    ) -> List[FlightSession]:
        return self._run_async(self.list_sessions_async(drone_id, start_after, limit))

    def log_telemetry(self, session_id: str, data: FlightData) -> None:
        self._run_async(self.log_telemetry_async(session_id, data))

    def log_telemetry_batch(self, session_id: str, data: List[FlightData]) -> None:
        self._run_async(self.log_telemetry_batch_async(session_id, data))

    def get_telemetry(
        self,
        session_id: str,
        start: Optional[datetime] = None,
        end: Optional[datetime] = None,
        limit: int = 10000,
    ) -> List[TelemetryRecord]:
        return self._run_async(self.get_telemetry_async(session_id, start, end, limit))

    def log_command(
        self,
        session_id: str,
        command: str,
        params: dict,
        result: int,
        duration_ms: float,
    ) -> None:
        self._run_async(self.log_command_async(session_id, command, params, result, duration_ms))

    def get_commands(
        self,
        session_id: str,
        command_filter: Optional[str] = None,
    ) -> List[CommandRecord]:
        return self._run_async(self.get_commands_async(session_id, command_filter))

    def close(self) -> None:
        self._run_async(self.close_async())

    async def __aenter__(self) -> "PostgresLogger":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.close_async()
