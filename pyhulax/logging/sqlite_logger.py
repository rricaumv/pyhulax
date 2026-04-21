"""
SQLite flight logger implementation.

Synchronous logging backend using sqlite3. Good for local development
and single-drone setups.
"""

import sqlite3
import json
import uuid
from datetime import datetime
from pathlib import Path
from typing import Optional, List

from ..core.models import FlightData
from .base import FlightLogger
from .models import FlightSession, TelemetryRecord, CommandRecord


class SQLiteLogger(FlightLogger):
    """
    SQLite-backed flight logger.

    Thread-safe for basic operations. Uses WAL mode for better
    concurrent read performance.
    """

    SCHEMA = """
    -- Flight sessions table
    CREATE TABLE IF NOT EXISTS sessions (
        session_id TEXT PRIMARY KEY,
        drone_id INTEGER NOT NULL,
        start_time TEXT NOT NULL,
        end_time TEXT,
        notes TEXT
    );

    -- Telemetry data table
    CREATE TABLE IF NOT EXISTS telemetry (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        session_id TEXT NOT NULL,
        timestamp TEXT NOT NULL,
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
        barrier INTEGER DEFAULT 0,
        FOREIGN KEY (session_id) REFERENCES sessions(session_id)
    );

    -- Commands table
    CREATE TABLE IF NOT EXISTS commands (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        session_id TEXT NOT NULL,
        timestamp TEXT NOT NULL,
        command TEXT NOT NULL,
        params TEXT,
        result INTEGER NOT NULL,
        duration_ms REAL NOT NULL,
        FOREIGN KEY (session_id) REFERENCES sessions(session_id)
    );

    -- Indexes for common queries
    CREATE INDEX IF NOT EXISTS idx_telemetry_session ON telemetry(session_id);
    CREATE INDEX IF NOT EXISTS idx_telemetry_timestamp ON telemetry(timestamp);
    CREATE INDEX IF NOT EXISTS idx_commands_session ON commands(session_id);
    CREATE INDEX IF NOT EXISTS idx_sessions_drone ON sessions(drone_id);
    CREATE INDEX IF NOT EXISTS idx_sessions_start ON sessions(start_time);
    """

    def __init__(self, db_path: str = "flight_logs.db"):
        """
        Initialize SQLite logger.

        Args:
            db_path: Path to SQLite database file
        """
        self.db_path = Path(db_path)
        self._init_db()

    def _init_db(self) -> None:
        """Initialize database schema."""
        with self._get_conn() as conn:
            conn.executescript(self.SCHEMA)
            # Enable WAL mode for better concurrent reads
            conn.execute("PRAGMA journal_mode=WAL")

    def _get_conn(self) -> sqlite3.Connection:
        """Get database connection with row factory."""
        conn = sqlite3.connect(self.db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        return conn

    # ==================== Session Management ====================

    def start_session(
        self,
        drone_id: int,
        notes: Optional[str] = None,
    ) -> str:
        """Start a new flight session."""
        session_id = str(uuid.uuid4())
        with self._get_conn() as conn:
            conn.execute(
                """INSERT INTO sessions (session_id, drone_id, start_time, notes)
                   VALUES (?, ?, ?, ?)""",
                (session_id, drone_id, datetime.now().isoformat(), notes)
            )
        return session_id

    def end_session(self, session_id: str) -> None:
        """End a flight session."""
        with self._get_conn() as conn:
            conn.execute(
                "UPDATE sessions SET end_time = ? WHERE session_id = ?",
                (datetime.now().isoformat(), session_id)
            )

    def get_session(self, session_id: str) -> Optional[FlightSession]:
        """Get session metadata."""
        with self._get_conn() as conn:
            row = conn.execute(
                "SELECT * FROM sessions WHERE session_id = ?",
                (session_id,)
            ).fetchone()

        if row is None:
            return None

        return FlightSession(
            session_id=row["session_id"],
            drone_id=row["drone_id"],
            start_time=datetime.fromisoformat(row["start_time"]),
            end_time=datetime.fromisoformat(row["end_time"]) if row["end_time"] else None,
            notes=row["notes"],
        )

    def list_sessions(
        self,
        drone_id: Optional[int] = None,
        start_after: Optional[datetime] = None,
        limit: int = 100,
    ) -> List[FlightSession]:
        """List flight sessions."""
        query = "SELECT * FROM sessions WHERE 1=1"
        params: list = []

        if drone_id is not None:
            query += " AND drone_id = ?"
            params.append(drone_id)

        if start_after is not None:
            query += " AND start_time > ?"
            params.append(start_after.isoformat())

        query += " ORDER BY start_time DESC LIMIT ?"
        params.append(limit)

        with self._get_conn() as conn:
            rows = conn.execute(query, params).fetchall()

        return [
            FlightSession(
                session_id=row["session_id"],
                drone_id=row["drone_id"],
                start_time=datetime.fromisoformat(row["start_time"]),
                end_time=datetime.fromisoformat(row["end_time"]) if row["end_time"] else None,
                notes=row["notes"],
            )
            for row in rows
        ]

    # ==================== Telemetry Logging ====================

    def log_telemetry(self, session_id: str, data: FlightData) -> None:
        """Log a telemetry data point."""
        with self._get_conn() as conn:
            conn.execute(
                """INSERT INTO telemetry
                   (session_id, timestamp, pos_x, pos_y, pos_z,
                    vel_x, vel_y, vel_z, yaw, pitch, roll,
                    altitude, battery, barrier)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    session_id,
                    data.timestamp.isoformat(),
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
            )

    def log_telemetry_batch(
        self,
        session_id: str,
        data: List[FlightData],
    ) -> None:
        """Log multiple telemetry data points in batch."""
        with self._get_conn() as conn:
            conn.executemany(
                """INSERT INTO telemetry
                   (session_id, timestamp, pos_x, pos_y, pos_z,
                    vel_x, vel_y, vel_z, yaw, pitch, roll,
                    altitude, battery, barrier)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                [
                    (
                        session_id,
                        d.timestamp.isoformat(),
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

    def get_telemetry(
        self,
        session_id: str,
        start: Optional[datetime] = None,
        end: Optional[datetime] = None,
        limit: int = 10000,
    ) -> List[TelemetryRecord]:
        """Retrieve telemetry for a session."""
        query = "SELECT * FROM telemetry WHERE session_id = ?"
        params: list = [session_id]

        if start is not None:
            query += " AND timestamp >= ?"
            params.append(start.isoformat())

        if end is not None:
            query += " AND timestamp <= ?"
            params.append(end.isoformat())

        query += " ORDER BY timestamp ASC LIMIT ?"
        params.append(limit)

        with self._get_conn() as conn:
            rows = conn.execute(query, params).fetchall()

        return [
            TelemetryRecord(
                id=row["id"],
                session_id=row["session_id"],
                timestamp=datetime.fromisoformat(row["timestamp"]),
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

    # ==================== Command Logging ====================

    def log_command(
        self,
        session_id: str,
        command: str,
        params: dict,
        result: int,
        duration_ms: float,
    ) -> None:
        """Log a command execution."""
        with self._get_conn() as conn:
            conn.execute(
                """INSERT INTO commands
                   (session_id, timestamp, command, params, result, duration_ms)
                   VALUES (?, ?, ?, ?, ?, ?)""",
                (
                    session_id,
                    datetime.now().isoformat(),
                    command,
                    json.dumps(params),
                    result,
                    duration_ms,
                )
            )

    def get_commands(
        self,
        session_id: str,
        command_filter: Optional[str] = None,
    ) -> List[CommandRecord]:
        """Retrieve commands for a session."""
        query = "SELECT * FROM commands WHERE session_id = ?"
        params: list = [session_id]

        if command_filter is not None:
            query += " AND command LIKE ?"
            params.append(f"%{command_filter}%")

        query += " ORDER BY timestamp ASC"

        with self._get_conn() as conn:
            rows = conn.execute(query, params).fetchall()

        return [
            CommandRecord(
                id=row["id"],
                session_id=row["session_id"],
                timestamp=datetime.fromisoformat(row["timestamp"]),
                command=row["command"],
                params=json.loads(row["params"]) if row["params"] else {},
                result=row["result"],
                duration_ms=row["duration_ms"],
            )
            for row in rows
        ]

    # ==================== Cleanup ====================

    def close(self) -> None:
        """Close database connection."""
        # SQLite connections are managed per-call, nothing to close
        pass

    def __enter__(self) -> "SQLiteLogger":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
