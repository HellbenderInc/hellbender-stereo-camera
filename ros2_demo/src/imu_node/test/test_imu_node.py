"""Tests for imu_node configuration."""

import json
import tempfile
from pathlib import Path

from imu_node.config import NodeConfig


def test_node_config_defaults() -> None:
    """Verify NodeConfig defaults."""
    config = NodeConfig()
    assert config.zenoh_key == "hb/stereo/imu/stream"
    assert config.zenoh_endpoints == []
    assert config.zenoh_mode == "peer"
    assert config.frame_id == "imu_link"
    assert len(config.angular_velocity_covariance) == 9
    assert config.angular_velocity_covariance == [0.0] * 9
    assert len(config.linear_acceleration_covariance) == 9
    assert config.linear_acceleration_covariance == [0.0] * 9


def test_node_config_load_from_missing_file() -> None:
    """Verify NodeConfig loads defaults when file is missing."""
    config = NodeConfig.load_from_file(Path("/nonexistent/path.json"))
    assert config.zenoh_key == "hb/stereo/imu/stream"
    assert config.frame_id == "imu_link"


def test_node_config_load_from_file() -> None:
    """Verify NodeConfig loads correctly from a JSON file."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        config_data = {
            "zenoh_key": "robot/imu",
            "zenoh_endpoints": ["tcp/router:7447"],
            "zenoh_mode": "client",
            "frame_id": "base_imu",
        }
        json.dump(config_data, f)
        temp_path = Path(f.name)

    try:
        config = NodeConfig.load_from_file(temp_path)
        assert config.zenoh_key == "robot/imu"
        assert config.zenoh_endpoints == ["tcp/router:7447"]
        assert config.zenoh_mode == "client"
        assert config.frame_id == "base_imu"
    finally:
        temp_path.unlink()


def test_node_config_load_partial_file() -> None:
    """Verify NodeConfig falls back to defaults for a malformed file."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write("{invalid json")
        temp_path = Path(f.name)

    try:
        config = NodeConfig.load_from_file(temp_path)
        assert config.zenoh_key == "hb/stereo/imu/stream"
    finally:
        temp_path.unlink()
