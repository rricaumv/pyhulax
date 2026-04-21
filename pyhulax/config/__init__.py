"""
Centralized configuration for drone_api.

Default SDK settings are loaded from package-shipped JSON files in
`pyhulax/config/`. User-supplied config values are merged over those defaults,
but only for fields that were explicitly provided.
"""

from __future__ import annotations

from functools import lru_cache
import json
from importlib import resources
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


IGNORED_MODEL_TYPES = (type(lambda: None), classmethod, staticmethod, property)


class ConfigModel(BaseModel):
    """Base class for immutable configuration models."""

    model_config = ConfigDict(frozen=True, ignored_types=IGNORED_MODEL_TYPES)


class NetworkConfig(ConfigModel):
    """Network connection settings."""

    drone_ip: str = "192.168.100.1"
    tcp_port: int = 8888
    udp_command_port: int = 8085
    udp_status_port: int = 8668
    udp_optitrack_port: int = 8688
    rtp_base_port: int = 9000
    web_port: int = 5000
    http_port: int = 12346


class ProtocolConfig(ConfigModel):
    """MAVLink protocol settings."""

    command_protocol: str = "tcp"
    serial_baudrate: int = 921600
    mavlink_system_id: int = 1
    mavlink_component_id: int = 2
    mavlink_component_file_id: int = 1


class DronePhysicsConfig(ConfigModel):
    """Drone physical dimensions (F09-lite: 189.3 x 184.6 x 51.4 mm)."""

    drone_width_cm: float = 18.93
    drone_depth_cm: float = 18.46
    min_altitude_cm: float = 30.0
    max_altitude_cm: float = 200.0


class FlightConfig(ConfigModel):
    """Default flight parameters."""

    default_takeoff_height_cm: int = 80
    default_flight_height_cm: int = 100
    default_speed_cms: int = 100
    position_tolerance_cm: float = 5.0
    yaw_tolerance_deg: float = 3.0


class MazePhysicsConfig(ConfigModel):
    """Maze grid physical constants (in cm)."""

    cell_size_cm: float = 60.0
    cell_offset_cm: float = 15.0
    qr_size_cm: float = 20.0
    qr_spacing_cm: float = 30.0
    padding_cm: float = 10.0
    safety_margin_cm: float = 5.0
    waypoint_tolerance_cm: float = 3.0
    wall_thickness_cm: float = 1.0


class ControllerConfig(ConfigModel):
    """PD flight controller gains and limits."""

    kp_xy: float = 2.0
    kd_xy: float = 0.5
    kp_z: float = 3.0
    kd_z: float = 0.8
    kp_yaw: float = 5.0
    kd_yaw: float = 1.0
    max_horizontal_output: float = 800
    max_vertical_output: float = 600
    max_yaw_output: float = 500
    control_rate_hz: float = 20.0


class VideoConfig(ConfigModel):
    """Video streaming configuration."""

    timeout_sec: float = 30.0
    buffer_size: int = 10
    jpeg_quality: int = 80
    max_fps: float = 30.0
    detection_confidence: float = 0.5
    nms_iou_threshold: float = 0.45


class TimeoutConfig(ConfigModel):
    """Timeout values for various operations."""

    command_timeout_sec: float = 4.0
    tcp_connect_timeout_sec: float = 5.0
    tcp_recv_timeout_sec: float = 1.0
    udp_timeout_sec: float = 1.0
    fly_to_timeout_sec: float = 30.0


class BatteryConfig(ConfigModel):
    """Battery level thresholds."""

    warning_threshold: int = 15
    critical_threshold: int = 10
    min_operational_threshold: int = 20


class MediaConfig(ConfigModel):
    """Default local storage paths for downloaded media."""

    base_dir: Path | str = Path("media")
    photo_dir: Path | str | None = None
    video_dir: Path | str | None = None
    log_dir: Path | str | None = None


class DroneConfig(ConfigModel):
    """Root configuration containing all config sections."""

    network: NetworkConfig = Field(default_factory=NetworkConfig)
    protocol: ProtocolConfig = Field(default_factory=ProtocolConfig)
    physics: DronePhysicsConfig = Field(default_factory=DronePhysicsConfig)
    flight: FlightConfig = Field(default_factory=FlightConfig)
    maze: MazePhysicsConfig = Field(default_factory=MazePhysicsConfig)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)
    video: VideoConfig = Field(default_factory=VideoConfig)
    timeouts: TimeoutConfig = Field(default_factory=TimeoutConfig)
    battery: BatteryConfig = Field(default_factory=BatteryConfig)
    media: MediaConfig = Field(default_factory=MediaConfig)


CONFIG_SECTION_MODELS: dict[str, type[ConfigModel]] = {
    "network": NetworkConfig,
    "protocol": ProtocolConfig,
    "physics": DronePhysicsConfig,
    "flight": FlightConfig,
    "maze": MazePhysicsConfig,
    "controller": ControllerConfig,
    "video": VideoConfig,
    "timeouts": TimeoutConfig,
    "battery": BatteryConfig,
    "media": MediaConfig,
}


def _load_section_defaults(section_name: str) -> dict[str, Any]:
    """Load one config section from the packaged config directory."""
    config_path = resources.files(__package__).joinpath(f"{section_name}.json")
    with config_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


@lru_cache(maxsize=1)
def load_default_config() -> DroneConfig:
    """Load the packaged default SDK config from `pyhulax/config/`."""
    config_data = {
        section_name: _load_section_defaults(section_name)
        for section_name in CONFIG_SECTION_MODELS
    }
    return DroneConfig.model_validate(config_data)


def merge_config(base_config: DroneConfig, override_config: DroneConfig) -> DroneConfig:
    """Merge explicit override fields into a base config recursively."""
    return _merge_model(base_config, override_config)


def _merge_model(base_model: BaseModel, override_model: BaseModel) -> BaseModel:
    """Merge explicitly-set fields from one pydantic model onto another."""
    updates: dict[str, Any] = {}

    for field_name in override_model.model_fields_set:
        override_value = getattr(override_model, field_name)
        base_value = getattr(base_model, field_name)

        if isinstance(base_value, BaseModel) and isinstance(override_value, BaseModel):
            updates[field_name] = _merge_model(base_value, override_value)
        else:
            updates[field_name] = override_value

    return base_model.model_copy(update=updates)


DEFAULT_CONFIG = load_default_config()


def get_config() -> DroneConfig:
    """Get the default SDK configuration loaded from the packaged config files."""
    return load_default_config()


def resolve_config(config: DroneConfig | None = None) -> DroneConfig:
    """Merge an optional override config over the packaged defaults."""
    base_config = get_config()
    if config is None:
        return base_config
    return merge_config(base_config, config)
