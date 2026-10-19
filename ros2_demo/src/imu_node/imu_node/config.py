"""Node configuration loader for imu_node."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

LOGGER = logging.getLogger("imu-node")


@dataclass
class NodeConfig:
    """Configuration for the IMU bridge node."""

    # Zenoh key expression (topic) on which the upstream device publishes
    # JSON-encoded ImuSample messages. The node opens a subscriber on this key.
    zenoh_key: str = "hb/stereo/imu/stream"

    # Optional list of Zenoh router/peer endpoints, e.g. ["tcp/router:7447"].
    # Empty list means rely on auto-discovery / default config.
    zenoh_endpoints: list[str] = field(default_factory=list)

    # Zenoh session mode: "peer" or "client".
    zenoh_mode: str = "peer"

    # frame_id stamped on every published sensor_msgs/Imu message.
    frame_id: str = "imu_link"

    # If True, derive roll and pitch from the accelerometer (gravity-vector
    # tilt estimate) and publish them as the Imu.orientation quaternion. Yaw
    # cannot be recovered from accel alone and is fixed at 0. This estimate is
    # only meaningful when the device is approximately stationary, since any
    # linear acceleration biases the gravity reading.
    estimate_orientation_from_accel: bool = True

    # Reference accelerometer direction when the device is at rest and level,
    # expressed in sensor-body axes. This sensor reads ~+9.8 m/s^2 along its
    # Y axis at rest, so the "up" reference is +Y. Adjust to [0, 0, 1] for a
    # REP-103 Z-up sensor, or e.g. [0, -1, 0] if the chip reports specific
    # force with the opposite sign.
    gravity_reference_axis: list[float] = field(default_factory=lambda: [0.0, 1.0, 0.0])

    # The ImuSample schema does not carry covariance, so these defaults are
    # always used to populate the sensor_msgs/Imu covariance matrices
    # (row-major 3x3). All zeros signals "covariance unknown" per the ROS
    # message spec (distinct from -1 which means "no measurement").
    angular_velocity_covariance: list[float] = field(
        default_factory=lambda: [0.0] * 9
    )
    linear_acceleration_covariance: list[float] = field(
        default_factory=lambda: [0.0] * 9
    )

    @staticmethod
    def load_from_file(config_path: Path) -> NodeConfig:
        """Load configuration from a JSON file, using defaults for missing keys.

        Args:
            config_path: Path to the JSON config file.

        Returns:
            NodeConfig populated from the file, or defaults if the file is
            absent or malformed.
        """
        if not config_path.is_file():
            LOGGER.info("Config file not found at %s, using defaults", config_path)
            return NodeConfig()

        try:
            with config_path.open() as f:
                config_dict = json.load(f)
            LOGGER.info("Loaded config from %s", config_path)
            return NodeConfig(**config_dict)
        except (json.JSONDecodeError, TypeError) as e:
            LOGGER.warning(
                "Error loading config from %s: %s. Using defaults.",
                config_path,
                e,
            )
            return NodeConfig()
