"""
Database models for flight logging.

These Pydantic models define the structure of logged data.
"""

from datetime import datetime
from typing import Optional, Any

from pydantic import BaseModel, ConfigDict, Field

from ..core.models import FlightData


IGNORED_MODEL_TYPES = (type(lambda: None), classmethod, staticmethod, property)


class LoggingModel(BaseModel):
    """Base logging model compatible with source and Cython builds."""

    model_config = ConfigDict(from_attributes=True, ignored_types=IGNORED_MODEL_TYPES)


class FlightSession(LoggingModel):
    """A recorded flight session."""
    session_id: str = Field(description="Unique session identifier (UUID)")
    drone_id: int = Field(description="Drone ID")
    start_time: datetime = Field(description="Session start time")
    end_time: Optional[datetime] = Field(default=None, description="Session end time")
    notes: Optional[str] = Field(default=None, description="Session notes")

class TelemetryRecord(LoggingModel):
    """A single telemetry data point."""
    id: Optional[int] = Field(default=None, description="Database record ID")
    session_id: str = Field(description="Parent session ID")
    timestamp: datetime = Field(description="Record timestamp")
    pos_x: float = Field(description="X position (cm)")
    pos_y: float = Field(description="Y position (cm)")
    pos_z: float = Field(description="Z position (cm)")
    vel_x: float = Field(description="X velocity (cm/s)")
    vel_y: float = Field(description="Y velocity (cm/s)")
    vel_z: float = Field(description="Z velocity (cm/s)")
    yaw: float = Field(description="Yaw angle (degrees)")
    pitch: float = Field(description="Pitch angle (degrees)")
    roll: float = Field(description="Roll angle (degrees)")
    altitude: float = Field(description="ToF altitude (cm)")
    battery: int = Field(description="Battery percentage")
    barrier: int = Field(default=0, description="Obstacle detection bitmask")
    @classmethod
    def from_flight_data(cls, session_id: str, data: FlightData) -> "TelemetryRecord":
        """Create TelemetryRecord from FlightData model."""
        return cls(
            session_id=session_id,
            timestamp=data.timestamp,
            pos_x=data.position.x,
            pos_y=data.position.y,
            pos_z=data.position.z,
            vel_x=data.velocity.x,
            vel_y=data.velocity.y,
            vel_z=data.velocity.z,
            yaw=data.orientation.yaw,
            pitch=data.orientation.pitch,
            roll=data.orientation.roll,
            altitude=data.altitude_tof,
            battery=data.battery_percent,
            barrier=data.barrier,
        )


class CommandRecord(LoggingModel):
    """A recorded command execution."""
    id: Optional[int] = Field(default=None, description="Database record ID")
    session_id: str = Field(description="Parent session ID")
    timestamp: datetime = Field(description="Command timestamp")
    command: str = Field(description="Command name")
    params: dict[str, Any] = Field(default_factory=dict, description="Command parameters")
    result: int = Field(description="Result code")
    duration_ms: float = Field(description="Command duration in milliseconds")
