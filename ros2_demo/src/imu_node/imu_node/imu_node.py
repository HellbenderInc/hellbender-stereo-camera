"""IMU bridge ROS 2 node.

Subscribes to a Zenoh key carrying JSON-encoded IMU samples, parses each
sample into a sensor_msgs/Imu message, and republishes both:
    - the typed sensor_msgs/Imu message on the configured ROS topic, and
    - the raw JSON payload as a std_msgs/String on a parallel topic.

The expected JSON schema matches ImuSample emitted by the upstream stereo
device on key ``hb/stereo/imu/stream``:

        {
                "accel":        {"x": <f32>, "y": <f32>, "z": <f32>},  # m/s^2, optional
                "gyro":         {"x": <f32>, "y": <f32>, "z": <f32>},  # rad/s, optional
                "timestamp_us": <u64>                                    # device clock, microseconds
        }
The schema can also be queried at ``hb/stereo/imu/stream/_schema``.
The schema does not carry orientation directly. The published
``sensor_msgs/Imu.orientation`` is therefore derived by this node only when
orientation estimation is enabled; otherwise it remains zeroed and
``orientation_covariance[0]`` is set to -1 per ROS convention for "no
estimate". ``frame_id`` is taken from the node config.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any

import rclpy
import zenoh
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from imu_node.config import NodeConfig


class ImuBridgeNode(Node):
    """ROS 2 node that bridges Zenoh JSON IMU samples onto ROS topics."""

    def __init__(self) -> None:
        """Initialize the IMU bridge node."""
        super().__init__("imu_node")

        # Declare ROS parameters
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("raw_json_topic", "/imu/raw_json")
        self.declare_parameter("config_file", "/etc/imu_node_config.json")

        imu_topic: str = self.get_parameter("imu_topic").value
        raw_json_topic: str = self.get_parameter("raw_json_topic").value
        config_file: str = self.get_parameter("config_file").value

        # Load node configuration
        self.config = NodeConfig.load_from_file(Path(config_file))

        self.message_count = 0
        self.last_message_time_s: float | None = None

        # Sensor-data style QoS: best-effort, keep-last(1).
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.imu_publisher = self.create_publisher(Imu, imu_topic, output_qos)
        self.raw_publisher = self.create_publisher(String, raw_json_topic, output_qos)

        # Periodic diagnostics so silent stalls are visible in logs.
        self.health_timer = self.create_timer(5.0, self._health_callback)

        # Start the Zenoh session and subscribe.
        self._init_zenoh()

        self.get_logger().info(
            "IMU bridge node initialized. "
            f"Zenoh key: {self.config.zenoh_key} | "
            f"IMU out: {imu_topic} | "
            f"Raw JSON out: {raw_json_topic}"
        )
        self.get_logger().info(
            "ROS env: "
            f"RMW_IMPLEMENTATION={os.getenv('RMW_IMPLEMENTATION', '<unset>')} | "
            f"ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '<unset>')} | "
            f"ROS_LOCALHOST_ONLY={os.getenv('ROS_LOCALHOST_ONLY', '<unset>')}"
        )

    def _init_zenoh(self) -> None:
        """Open a Zenoh session and subscribe to the configured key."""
        zenoh_config = zenoh.Config()
        zenoh_config.insert_json5("mode", json.dumps(self.config.zenoh_mode))
        if self.config.zenoh_endpoints:
            zenoh_config.insert_json5(
                "connect/endpoints", json.dumps(self.config.zenoh_endpoints)
            )

        self.zenoh_session = zenoh.open(zenoh_config)
        self.zenoh_subscriber = self.zenoh_session.declare_subscriber(
            self.config.zenoh_key, self._zenoh_callback
        )

    def _zenoh_callback(self, sample: Any) -> None:
        """Handle a single Zenoh sample carrying a JSON IMU payload.

        Args:
            sample: Zenoh sample. ``sample.payload`` is bytes containing UTF-8
                    JSON.
        """
        try:
            payload_bytes = bytes(sample.payload)
            payload_str = payload_bytes.decode("utf-8")
        except (UnicodeDecodeError, AttributeError) as exc:
            self.get_logger().error(f"Failed to decode Zenoh payload: {exc}")
            return

        # Always republish the raw JSON, even if parsing fails downstream.
        raw_msg = String()
        raw_msg.data = payload_str
        self.raw_publisher.publish(raw_msg)

        try:
            data = json.loads(payload_str)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid JSON payload, skipping Imu publish: {exc}")
            return

        try:
            imu_msg = self._json_to_imu(data)
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Could not build Imu from JSON: {exc}")
            return

        self.imu_publisher.publish(imu_msg)

        self.message_count += 1
        self.last_message_time_s = time.time()

        if self.message_count % 100 == 0:
            self.get_logger().info(f"Forwarded {self.message_count} IMU samples")

    def _json_to_imu(self, data: dict[str, Any]) -> Imu:
        """Convert a parsed ImuSample JSON payload into sensor_msgs/Imu."""
        imu = Imu()

        # Header: stamp from device microseconds, fall back to ROS clock.
        ts_us = data.get("timestamp_us")
        if isinstance(ts_us, (int, float)):
            ts_ns = int(ts_us) * 1000
            imu.header.stamp.sec = ts_ns // 1_000_000_000
            imu.header.stamp.nanosec = ts_ns % 1_000_000_000
        else:
            imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.config.frame_id

        # Angular velocity (rad/s)
        gyro = data.get("gyro") or {}
        imu.angular_velocity.x = float(gyro.get("x", 0.0))
        imu.angular_velocity.y = float(gyro.get("y", 0.0))
        imu.angular_velocity.z = float(gyro.get("z", 0.0))

        # Linear acceleration (m/s^2)
        accel = data.get("accel") or {}
        ax = float(accel.get("x", 0.0))
        ay = float(accel.get("y", 0.0))
        az = float(accel.get("z", 0.0))
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Orientation: derive tilt from the gravity vector if enabled. The
        # rotation about the gravity axis itself (yaw, in the world-up sense)
        # is unobservable from a single accel sample.
        if self.config.estimate_orientation_from_accel and accel:
            qx, qy, qz, qw = self._tilt_quaternion_from_accel(
                ax, ay, az, self.config.gravity_reference_axis
            )
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
            # Covariance unknown — all zeros per ROS convention.
            imu.orientation_covariance = [0.0] * 9
        else:
            # No estimate available: zero quaternion + covariance[0] = -1.
            imu.orientation.x = 0.0
            imu.orientation.y = 0.0
            imu.orientation.z = 0.0
            imu.orientation.w = 0.0
            imu.orientation_covariance = [-1.0] + [0.0] * 8

        imu.angular_velocity_covariance = list(self.config.angular_velocity_covariance)
        imu.linear_acceleration_covariance = list(
            self.config.linear_acceleration_covariance
        )

        return imu

    @staticmethod
    def _tilt_quaternion_from_accel(
        ax: float,
        ay: float,
        az: float,
        gravity_reference: list[float],
    ) -> tuple[float, float, float, float]:
        """Shortest-arc quaternion from the at-rest reference axis to the
        measured accel direction.

        Treats the accel reading as the body-frame gravity vector. Returns the
        quaternion that rotates ``gravity_reference`` (the body-frame "up"
        direction when level) into the unit accel vector. Rotation about the
        gravity axis itself is unobservable from a single sample and is left
        at zero (shortest-arc choice).

        Args:
            ax, ay, az: Accelerometer sample in m/s^2.
            gravity_reference: Body-frame axis that points up when the device
                is level (e.g. ``[0, 1, 0]`` for a Y-up mount).

        Returns:
            Unit quaternion (qx, qy, qz, qw). Returns identity if either
            vector is degenerate.
        """
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        rx, ry, rz = gravity_reference
        r_norm = math.sqrt(rx * rx + ry * ry + rz * rz)
        if a_norm < 1e-6 or r_norm < 1e-6:
            return (0.0, 0.0, 0.0, 1.0)

        # Normalize both vectors.
        ax_n, ay_n, az_n = ax / a_norm, ay / a_norm, az / a_norm
        rx_n, ry_n, rz_n = rx / r_norm, ry / r_norm, rz / r_norm

        dot = rx_n * ax_n + ry_n * ay_n + rz_n * az_n

        # Anti-parallel: 180-degree rotation about any axis perpendicular to r.
        if dot < -1.0 + 1e-6:
            # Pick the world basis axis least aligned with r to find a
            # perpendicular direction.
            if abs(rx_n) <= abs(ry_n) and abs(rx_n) <= abs(rz_n):
                ortho = (1.0, 0.0, 0.0)
            elif abs(ry_n) <= abs(rz_n):
                ortho = (0.0, 1.0, 0.0)
            else:
                ortho = (0.0, 0.0, 1.0)
            # axis = r x ortho, normalized
            cx = ry_n * ortho[2] - rz_n * ortho[1]
            cy = rz_n * ortho[0] - rx_n * ortho[2]
            cz = rx_n * ortho[1] - ry_n * ortho[0]
            cn = math.sqrt(cx * cx + cy * cy + cz * cz)
            return (cx / cn, cy / cn, cz / cn, 0.0)

        # Standard shortest-arc construction.
        cx = ry_n * az_n - rz_n * ay_n
        cy = rz_n * ax_n - rx_n * az_n
        cz = rx_n * ay_n - ry_n * ax_n
        qw = 1.0 + dot
        qx, qy, qz = cx, cy, cz
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return (qx / n, qy / n, qz / n, qw / n)

    def _health_callback(self) -> None:
        """Emit periodic health status for easier runtime debugging."""
        if self.last_message_time_s is None:
            self.get_logger().warning(
                f"No Zenoh samples received yet on {self.config.zenoh_key}"
            )
            return

        seconds_since_last = time.time() - self.last_message_time_s
        if seconds_since_last > 5.0:
            self.get_logger().warning(
                f"No IMU sample for {seconds_since_last:.1f}s on "
                f"{self.config.zenoh_key}"
            )

    def destroy_node(self) -> bool:
        """Close the Zenoh session before destroying the ROS node."""
        try:
            if hasattr(self, "zenoh_subscriber"):
                self.zenoh_subscriber.undeclare()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Error undeclaring Zenoh subscriber: {exc}")
        try:
            if hasattr(self, "zenoh_session"):
                self.zenoh_session.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Error closing Zenoh session: {exc}")
        return super().destroy_node()


def main(args: "list[str] | None" = None) -> None:
    """Main entry point for the ROS node.

    Args:
        args: Command line arguments.
    """
    rclpy.init(args=args)
    node = ImuBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
