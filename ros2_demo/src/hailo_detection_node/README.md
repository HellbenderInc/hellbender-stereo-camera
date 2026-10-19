# Hailo Person Detection Node

[![build - Hatch](https://img.shields.io/badge/%F0%9F%A5%9A-Hatch-4051b5.svg)](https://github.com/pypa/hatch)
[![linting - Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/charliermarsh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
[![analysis - Pyright](https://img.shields.io/badge/analysis-Pyright-blue.svg)](https://github.com/microsoft/pyright)

-----

A ROS 2 Python node that performs real-time person detection on stereo color+depth image pairs using the Hailo-8 accelerator and YOLO inference. It is designed to work with the `stereo_depth_node` package in this repository.

## Features

- **Real-time inference**: Hailo-accelerated YOLO person detection
- **Stereo awareness**: Processes rectified color+depth image pairs to provide depth estimates for detected persons
- **Latency metrics**: Publishes per-frame pipeline latency breakdown via ROS DiagnosticArray
- **JPEG compression**: Outputs annotated images as JPEG CompressedImage messages for bandwidth efficiency
- **Configurable**: Runtime control over model naming, output node naming, color, line thickness, and depth range via JSON config

## Requirements

- ROS 2 Humble
- Hailo-8 PCIe accelerator with hailort SDK
- A compatible Hailo HEF model file and matching postprocess output node name
- Python >= 3.10
- OpenCV 4.0+

## Input/Output

### Subscriptions

- `/stereo/image` (`stereo_depth_node/msg/StereoImage`)
  - Contains: `left_image` (bgr8), `depth_image` (mono16)
  - Published by: `stereo_depth_node`

### Publications

- `/hailo/depth_annotated/compressed` (sensor_msgs/CompressedImage): Colorized depth map with person bounding boxes and depth labels (JPEG)
- `/hailo/rgb_annotated/compressed` (sensor_msgs/CompressedImage): Rectified RGB image with person bounding boxes and depth labels (JPEG)
- `/hailo/pipeline_latency` (diagnostic_msgs/DiagnosticArray): Per-frame latency metrics (preprocess, infer, annotate, publish, total)

## Installation

Requires Python >= 3.10. Recommend using a virtual environment:

```console
pip install .
```

Or from a wheel:

```console
pip install <filename>.whl
```

## Development

Build/test commands use `hatch`:

```shell
# Run tests
hatch test

# Test all Python versions
hatch test -a

# Test with coverage
hatch test -c

# Format, lint, and type-check
hatch fmt

# Build wheel
hatch build
```

## Docker

This package no longer carries its own Docker assets. Use the top-level
combined image in `ros2_demo/docker/` to build and run the full stack,
including Hailo runtime dependencies and the baked-in model assets.

## ROS 2 Usage

Launch the node with default parameters:

```shell
ros2 launch hailo_detection_node hailo_detection.launch.py
```

Launch with custom parameters:

```shell
ros2 launch hailo_detection_node hailo_detection.launch.py \
  stereo_image_topic:=/my/stereo/topic \
  output_depth_topic:=/my/depth/output \
  config_file:=/path/to/custom_config.json
```

### Launch Parameters

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `stereo_image_topic` | `/stereo/image` | Input topic: `stereo_depth_node/msg/StereoImage` |
| `output_depth_topic` | `/hailo/depth_annotated` | Annotated depth output (CompressedImage) |
| `output_rgb_topic` | `/hailo/rgb_annotated` | Annotated RGB output (CompressedImage) |
| `latency_topic` | `/hailo/pipeline_latency` | Latency metrics (DiagnosticArray) |
| `config_file` | `/etc/node_config.json` | Node config JSON file |
| `jpeg_quality` | `80` | JPEG compression quality (0-100) |

### Configuration (JSON)

The node is configured via a JSON file (default: `config/node_config.json`):

```json
{
  "model_path": "/networks",               // Directory containing HEF model files
  "model_name": "yolox_s_leaky",          // Model subdirectory and HEF filename stem
  "nms_output_node": "yolox_nms_postprocess", // Postprocess output node name from the HEF
  "person_center_ratio": 0.3,              // Use central 30% of bbox for depth median
  "min_depth_m": 0.3,                      // Depth range: minimum (must match stereo sender)
  "max_depth_m": 10.0,                     // Depth range: maximum
  "box_color_bgr": [255, 255, 255],        // Bounding box color (BGR)
  "box_thickness": 4,                      // Bounding box line width (pixels)
  "text_scale": 1.0,                       // Detection label text scale
  "text_thickness": 2                      // Detection label text thickness
}
```

NOTE: The ```yolox_s_leaky``` model file is copied from Hailo's public model [site](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst)

The full inference output key is derived at runtime as `model_name/nms_output_node`.
This lets you switch to a different exported model or postprocess node without
changing Python code.

For example, if your model exports detections under `my_model/custom_nms`, use:

```json
{
  "model_path": "/networks",
  "model_name": "my_model",
  "nms_output_node": "custom_nms"
}
```

## Monitoring

Check node health and endpoint discovery:

```shell
# View node info
ros2 node info /hailo_detection_node

# View output topics
ros2 topic list -t
ros2 topic info /hailo/depth_annotated

# Monitor latency metrics
ros2 topic echo /hailo/pipeline_latency
```

Check logs for errors and per-frame latency:

```shell
ros2 node list
ros2 node info /hailo_detection_node
```
