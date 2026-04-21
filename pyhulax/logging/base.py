"""
Abstract base class for flight loggers.

Defines the interface that all logger implementations must follow.
"""

from abc import ABC, abstractmethod
from datetime import datetime
from typing import Optional, List

from ..core.models import FlightData
from .models import FlightSession, TelemetryRecord, CommandRecord


class FlightLogger(ABC):
    """
    Abstract interface for flight logging backends.

    Implementations must provide both sync and async versions of methods.
    The sync versions can be simple wrappers around async versions using
    asyncio.run() for backends that are natively async.
    """

    # ==================== Session Management ====================

    @abstractmethod
    def start_session(
        self,
        drone_id: int,
        notes: Optional[str] = None,
    ) -> str:
        """
        Start a new flight session.

        Args:
            drone_id: Drone identifier
            notes: Optional session notes

        Returns:
            Session ID (UUID string)
        """
        pass

    @abstractmethod
    def end_session(self, session_id: str) -> None:
        """
        End a flight session.

        Args:
            session_id: Session to end
        """
        pass

    @abstractmethod
    def get_session(self, session_id: str) -> Optional[FlightSession]:
        """
        Get session metadata.

        Args:
            session_id: Session ID

        Returns:
            FlightSession or None if not found
        """
        pass

    @abstractmethod
    def list_sessions(
        self,
        drone_id: Optional[int] = None,
        start_after: Optional[datetime] = None,
        limit: int = 100,
    ) -> List[FlightSession]:
        """
        List flight sessions.

        Args:
            drone_id: Filter by drone ID
            start_after: Filter sessions after this time
            limit: Maximum sessions to return

        Returns:
            List of FlightSession
        """
        pass

    # ==================== Telemetry Logging ====================

    @abstractmethod
    def log_telemetry(self, session_id: str, data: FlightData) -> None:
        """
        Log a telemetry data point.

        Args:
            session_id: Session ID
            data: Flight data to log
        """
        pass

    @abstractmethod
    def log_telemetry_batch(
        self,
        session_id: str,
        data: List[FlightData],
    ) -> None:
        """
        Log multiple telemetry data points in batch.

        Args:
            session_id: Session ID
            data: List of flight data to log
        """
        pass

    @abstractmethod
    def get_telemetry(
        self,
        session_id: str,
        start: Optional[datetime] = None,
        end: Optional[datetime] = None,
        limit: int = 10000,
    ) -> List[TelemetryRecord]:
        """
        Retrieve telemetry for a session.

        Args:
            session_id: Session ID
            start: Start time filter
            end: End time filter
            limit: Maximum records to return

        Returns:
            List of TelemetryRecord
        """
        pass

    # ==================== Command Logging ====================

    @abstractmethod
    def log_command(
        self,
        session_id: str,
        command: str,
        params: dict,
        result: int,
        duration_ms: float,
    ) -> None:
        """
        Log a command execution.

        Args:
            session_id: Session ID
            command: Command name
            params: Command parameters
            result: Result code
            duration_ms: Execution duration in milliseconds
        """
        pass

    @abstractmethod
    def get_commands(
        self,
        session_id: str,
        command_filter: Optional[str] = None,
    ) -> List[CommandRecord]:
        """
        Retrieve commands for a session.

        Args:
            session_id: Session ID
            command_filter: Filter by command name (substring match)

        Returns:
            List of CommandRecord
        """
        pass

    # ==================== Cleanup ====================

    @abstractmethod
    def close(self) -> None:
        """Close database connection and release resources."""
        pass
