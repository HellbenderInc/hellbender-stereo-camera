"""Launch file for the IMU Zenoh bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the IMU bridge node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="Output topic: sensor_msgs/Imu derived from JSON",
            ),
            DeclareLaunchArgument(
                "raw_json_topic",
                default_value="/imu/raw_json",
                description="Output topic: std_msgs/String, raw JSON received over Zenoh",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="/etc/imu_node_config.json",
                description="Path to node config JSON file",
            ),
            Node(
                package="imu_node",
                executable="imu_node",
                name="imu_node",
                parameters=[
                    {
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "raw_json_topic": LaunchConfiguration("raw_json_topic"),
                        "config_file": LaunchConfiguration("config_file"),
                    }
                ],
                output="screen",
            ),
        ]
    )
