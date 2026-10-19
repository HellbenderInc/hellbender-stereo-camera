"""Unit tests for JSON-to-Imu conversion and tilt quaternion math."""

from __future__ import annotations

import importlib
import json
import math
import sys
import types
from types import SimpleNamespace


def _install_ros_stubs() -> None:
    """Install minimal ROS/Zenoh stubs so this module can be imported in pytest."""
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda *args, **kwargs: None
        rclpy.shutdown = lambda *args, **kwargs: None
        sys.modules["rclpy"] = rclpy

    if "rclpy.executors" not in sys.modules:
        executors = types.ModuleType("rclpy.executors")

        class MultiThreadedExecutor:
            def add_node(self, node: object) -> None:
                pass

            def spin(self) -> None:
                pass

        executors.MultiThreadedExecutor = MultiThreadedExecutor
        sys.modules["rclpy.executors"] = executors

    if "rclpy.node" not in sys.modules:
        node_mod = types.ModuleType("rclpy.node")

        class Node:
            def __init__(self, *args, **kwargs) -> None:
                pass

        node_mod.Node = Node
        sys.modules["rclpy.node"] = node_mod

    if "rclpy.qos" not in sys.modules:
        qos_mod = types.ModuleType("rclpy.qos")

        class QoSProfile:
            def __init__(self, *args, **kwargs) -> None:
                pass

        class ReliabilityPolicy:
            BEST_EFFORT = "best_effort"

        class DurabilityPolicy:
            VOLATILE = "volatile"

        qos_mod.QoSProfile = QoSProfile
        qos_mod.ReliabilityPolicy = ReliabilityPolicy
        qos_mod.DurabilityPolicy = DurabilityPolicy
        sys.modules["rclpy.qos"] = qos_mod

    if "sensor_msgs.msg" not in sys.modules:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

        class Imu:
            def __init__(self) -> None:
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )
                self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)
                self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.orientation_covariance = [0.0] * 9
                self.angular_velocity_covariance = [0.0] * 9
                self.linear_acceleration_covariance = [0.0] * 9

        sensor_msgs_msg.Imu = Imu
        sensor_msgs.msg = sensor_msgs_msg
        sys.modules["sensor_msgs"] = sensor_msgs
        sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    if "std_msgs.msg" not in sys.modules:
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")

        class String:
            def __init__(self) -> None:
                self.data = ""

        std_msgs_msg.String = String
        std_msgs.msg = std_msgs_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msgs_msg

    if "zenoh" not in sys.modules:
        zenoh = types.ModuleType("zenoh")

        class Config:
            def insert_json5(self, key: str, value: str) -> None:
                pass

        class _Session:
            def declare_subscriber(self, key: str, callback) -> object:
                return object()

            def close(self) -> None:
                pass

        zenoh.Config = Config
        zenoh.open = lambda cfg: _Session()
        sys.modules["zenoh"] = zenoh


def _import_imu_module():
    _install_ros_stubs()
    module = importlib.import_module("imu_node.imu_node")
    return importlib.reload(module)


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


class _Logger:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.infos: list[str] = []

    def error(self, msg: str) -> None:
        self.errors.append(msg)

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)

    def info(self, msg: str) -> None:
        self.infos.append(msg)


class _TestNodeHarness:
    def __init__(self, module) -> None:
        self._module = module
        self.config = SimpleNamespace(
            frame_id="imu_link",
            estimate_orientation_from_accel=True,
            gravity_reference_axis=[0.0, 1.0, 0.0],
            angular_velocity_covariance=[0.0] * 9,
            linear_acceleration_covariance=[0.0] * 9,
        )
        self.raw_publisher = _Publisher()
        self.imu_publisher = _Publisher()
        self.message_count = 0
        self.last_message_time_s = None
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=123, nanosec=456))
        )

    def _json_to_imu(self, data):
        return self._module.ImuBridgeNode._json_to_imu(self, data)

    def _tilt_quaternion_from_accel(self, ax, ay, az, gravity_reference):
        return self._module.ImuBridgeNode._tilt_quaternion_from_accel(
            ax, ay, az, gravity_reference
        )


class _Sample:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload


def test_json_to_imu_valid_sample() -> None:
    module = _import_imu_module()
    harness = _TestNodeHarness(module)

    imu_msg = module.ImuBridgeNode._json_to_imu(
        harness,
        {
            "timestamp_us": 1_500_000,
            "gyro": {"x": 0.1, "y": -0.2, "z": 0.3},
            "accel": {"x": 0.0, "y": 9.81, "z": 0.0},
        },
    )

    assert imu_msg.header.frame_id == "imu_link"
    assert imu_msg.header.stamp.sec == 1
    assert imu_msg.header.stamp.nanosec == 500_000_000
    assert imu_msg.angular_velocity.x == 0.1
    assert imu_msg.angular_velocity.y == -0.2
    assert imu_msg.angular_velocity.z == 0.3
    assert imu_msg.linear_acceleration.x == 0.0
    assert imu_msg.linear_acceleration.y == 9.81
    assert imu_msg.linear_acceleration.z == 0.0
    assert imu_msg.orientation_covariance == [0.0] * 9
    assert imu_msg.angular_velocity_covariance == [0.0] * 9
    assert imu_msg.linear_acceleration_covariance == [0.0] * 9


def test_zenoh_callback_invalid_json_payload_warns_without_imu_publish() -> None:
    module = _import_imu_module()
    harness = _TestNodeHarness(module)

    module.ImuBridgeNode._zenoh_callback(harness, _Sample(b"{not-valid-json"))

    assert len(harness.raw_publisher.messages) == 1
    assert len(harness.imu_publisher.messages) == 0
    assert any("Invalid JSON payload" in msg for msg in harness.get_logger().warnings)


def test_zenoh_callback_invalid_utf8_payload_logs_error() -> None:
    module = _import_imu_module()
    harness = _TestNodeHarness(module)

    module.ImuBridgeNode._zenoh_callback(harness, _Sample(b"\xff\xfe\xfd"))

    assert len(harness.raw_publisher.messages) == 0
    assert len(harness.imu_publisher.messages) == 0
    assert any("Failed to decode Zenoh payload" in msg for msg in harness.get_logger().errors)


def test_zenoh_callback_malformed_schema_skips_imu_publish() -> None:
    module = _import_imu_module()
    harness = _TestNodeHarness(module)

    bad_schema = json.dumps(
        {
            "timestamp_us": 1_000,
            "gyro": 3.14,
            "accel": {"x": 0.0, "y": 9.81, "z": 0.0},
        }
    ).encode("utf-8")
    module.ImuBridgeNode._zenoh_callback(harness, _Sample(bad_schema))

    assert len(harness.raw_publisher.messages) == 1
    assert len(harness.imu_publisher.messages) == 0
    assert any("Could not build Imu from JSON" in msg for msg in harness.get_logger().warnings)


def test_tilt_quaternion_parallel_returns_identity() -> None:
    module = _import_imu_module()

    qx, qy, qz, qw = module.ImuBridgeNode._tilt_quaternion_from_accel(
        0.0, 9.81, 0.0, [0.0, 1.0, 0.0]
    )

    assert qx == 0.0
    assert qy == 0.0
    assert qz == 0.0
    assert qw == 1.0


def test_tilt_quaternion_antiparallel_has_180_degree_rotation() -> None:
    module = _import_imu_module()

    qx, qy, qz, qw = module.ImuBridgeNode._tilt_quaternion_from_accel(
        0.0, -9.81, 0.0, [0.0, 1.0, 0.0]
    )

    vec_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    assert math.isclose(abs(qw), 0.0, abs_tol=1e-6)
    assert math.isclose(vec_norm, 1.0, rel_tol=1e-6)


def test_tilt_quaternion_zero_acceleration_returns_identity() -> None:
    module = _import_imu_module()

    qx, qy, qz, qw = module.ImuBridgeNode._tilt_quaternion_from_accel(
        0.0, 0.0, 0.0, [0.0, 1.0, 0.0]
    )

    assert qx == 0.0
    assert qy == 0.0
    assert qz == 0.0
    assert qw == 1.0
