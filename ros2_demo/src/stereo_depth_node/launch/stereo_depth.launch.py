from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the stereo depth + hailo detection nodes."""
    return LaunchDescription(
        [
            # ----- stereo_depth_node arguments -----
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/depth/image",
                description="Base topic name for depth (used for colorized visualization)",
            ),
            DeclareLaunchArgument(
                "calib_file",
                default_value="/etc/hb_calib_params.json",
                description="Path to stereo calibration JSON file",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="/etc/stereo_config.yaml",
                description="Path to stereo config YAML file",
            ),
            DeclareLaunchArgument(
                "baseline_m",
                default_value="0.14",
                description="Stereo baseline in meters",
            ),
            DeclareLaunchArgument(
                "gstreamer_pipeline",
                default_value="",
                description="GStreamer pipeline for video capture (empty = auto-build)",
            ),
            # ----- Optional node launches -----
            DeclareLaunchArgument(
                "launch_hailo",
                default_value="true",
                description="Launch the Hailo detection node (true/false)",
            ),
            DeclareLaunchArgument(
                "launch_imu",
                default_value="true",
                description="Launch the IMU node (true/false)",
            ),
            DeclareLaunchArgument(
                "launch_foxglove",
                default_value="true",
                description="Launch the Foxglove bridge for visualization (true/false)",
            ),
            DeclareLaunchArgument(
                "stereo_image_topic",
                default_value="/stereo/image",
                description="Combined stereo image topic (StereoImage msg)",
            ),
            DeclareLaunchArgument(
                "output_depth_topic",
                default_value="/hailo/depth_annotated",
                description="Output: colorized depth with person detections",
            ),
            DeclareLaunchArgument(
                "output_rgb_topic",
                default_value="/hailo/rgb_annotated",
                description="Output: RGB image with person detections",
            ),
            DeclareLaunchArgument(
                "latency_topic",
                default_value="/hailo/pipeline_latency",
                description="Output: DiagnosticArray latency metrics",
            ),
            DeclareLaunchArgument(
                "hailo_config_file",
                default_value="/etc/node_config.json",
                description="Path to Hailo node config JSON file",
            ),
            DeclareLaunchArgument(
                "imu_config_file",
                default_value="/etc/imu_config.yaml",
                description="Path to IMU node config YAML file",
            ),
            # ----- Nodes -----
            Node(
                package="stereo_depth_node",
                executable="stereo_depth_node",
                name="stereo_depth_node",
                parameters=[
                    {
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "stereo_image_topic": LaunchConfiguration("stereo_image_topic"),
                        "calib_file": LaunchConfiguration("calib_file"),
                        "config_file": LaunchConfiguration("config_file"),
                        "baseline_m": ParameterValue(
                            LaunchConfiguration("baseline_m"), value_type=float
                        ),
                        "gstreamer_pipeline": LaunchConfiguration(
                            "gstreamer_pipeline"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="hailo_detection_node",
                executable="hailo_detection_node",
                name="hailo_detection_node",
                condition=IfCondition(LaunchConfiguration("launch_hailo")),
                parameters=[
                    {
                        "stereo_image_topic": LaunchConfiguration(
                            "stereo_image_topic"
                        ),
                        "output_depth_topic": LaunchConfiguration(
                            "output_depth_topic"
                        ),
                        "output_rgb_topic": LaunchConfiguration(
                            "output_rgb_topic"
                        ),
                        "latency_topic": LaunchConfiguration("latency_topic"),
                        "config_file": LaunchConfiguration("hailo_config_file"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="imu_node",
                executable="imu_node",
                name="imu_node",
                condition=IfCondition(LaunchConfiguration("launch_imu")),
                parameters=[
                    {
                        "config_file": LaunchConfiguration("imu_config_file"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                condition=IfCondition(LaunchConfiguration("launch_foxglove")),
                parameters=[{
                    "port": 8765,
                    "address": "0.0.0.0",
                }],
                output="screen",
            ),
        ]
    )
