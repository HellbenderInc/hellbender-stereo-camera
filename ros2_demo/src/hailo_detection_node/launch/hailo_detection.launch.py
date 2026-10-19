"""Launch file for the Hailo person detection node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the Hailo detection node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stereo_image_topic",
                default_value="/stereo/image",
                description="Input topic: StereoImage message with left color and depth_image",
            ),
            DeclareLaunchArgument(
                "output_depth_topic",
                default_value="/hailo/depth_annotated",
                description="Output topic: CompressedImage (JPEG) depth map with person boxes",
            ),
            DeclareLaunchArgument(
                "output_rgb_topic",
                default_value="/hailo/rgb_annotated",
                description="Output base topic for RGB with person boxes; final CompressedImage (JPEG) topic is <output_rgb_topic>/compressed",
            ),
            DeclareLaunchArgument(
                "latency_topic",
                default_value="/hailo/pipeline_latency",
                description="Output topic: DiagnosticArray latency metrics",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="/etc/node_config.json",
                description="Path to node config JSON file",
            ),
            Node(
                package="hailo_detection_node",
                executable="hailo_detection_node",
                name="hailo_detection_node",
                parameters=[
                    {
                        "stereo_image_topic": LaunchConfiguration("stereo_image_topic"),
                        "output_depth_topic": LaunchConfiguration("output_depth_topic"),
                        "output_rgb_topic": LaunchConfiguration("output_rgb_topic"),
                        "latency_topic": LaunchConfiguration("latency_topic"),
                        "config_file": LaunchConfiguration("config_file"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                parameters=[{"port": 8765}],
                output="screen",
            ),
        ]
    )
