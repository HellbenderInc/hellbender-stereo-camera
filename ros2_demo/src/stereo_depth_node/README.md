# stereo_depth_node

ROS 2 Humble package for real-time stereo depth estimation from a side-by-side UYVY camera stream using OpenCV StereoSGBM or StereoBM.

The node publishes:

- A custom synchronized stereo message containing rectified left color image + depth image.
- Standalone depth and left-image topics.
- Optional colorized depth visualization.
- Optional point cloud.
- Pipeline latency diagnostics.

## Deploying the Combined Image to a Camera

These steps assume you have a saved image tarball (for example a `.tar.gz`
release asset produced from the top-level `ros2_demo/docker/Dockerfile`) and
shell access to the camera.

> `sudo` is required on the camera unless the deploying user is a member of the `docker` group.

1. Copy the image tarball to the camera:

    ```bash
    scp hb-stereo-ros2-demo-<tag-or-sha>-linux-arm64.tar.gz user@camera:/tmp/
    ```

2. On the camera, decompress and load the image:

    ```bash
    gunzip -c /tmp/hb-stereo-ros2-demo-<tag-or-sha>-linux-arm64.tar.gz | sudo docker load
    ```

3. Export the tag or SHA that was loaded from the archive:

    ```bash
    export HB_STEREO_ROS2_DEMO_TAG=<tag-or-sha>
    ```

4. Verify the files are in place:

    ```bash
    ls /etc/hb_calib_params.json /etc/hb-camera-config/
    ```

    The container installs missing default configs into `/etc` on first start.
    `hb_calib_params.json` must still be provided by the host.

5. Start the stack via `docker compose` (from the directory containing `docker-compose.yml`):

    ```bash
    cd ros2_demo
    sudo docker compose up -d
    ```

## Package Overview

- Package name: `stereo_depth_node`
- Language: C++
- Build type: `ament_cmake`
- ROS distribution target: Humble

Key sources:

- `src/stereo_depth_node.cpp`: Node lifecycle, capture loop, rectification, disparity, depth, publishing.
- `src/stereo_proc.cpp`: Stereo processing helpers.
- `src/stereo_demo_config.cpp`: Runtime config file loading.
- `src/hb_cal_params.cpp`: Calibration JSON loading and scaling.

## Requirements

### System

- Ubuntu-based ROS 2 environment (native or container).
- ROS 2 Humble.
- OpenCV with GStreamer support.
- Camera delivering side-by-side UYVY frames (left/right packed horizontally).

### ROS dependencies

Declared in `package.xml`:

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `diagnostic_msgs`
- `rosidl_default_generators`
- `rosidl_default_runtime`

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select stereo_depth_node
source install/setup.bash
```

## Run

### Node only

```bash
ros2 run stereo_depth_node stereo_depth_node
```

### Launch file

```bash
ros2 launch stereo_depth_node stereo_depth.launch.py
```

The launch file can also start optional companion nodes if those packages are installed in the same workspace:

- `hailo_detection_node`
- `imu_node`
- `foxglove_bridge`

Disable optional nodes with launch arguments:

```bash
ros2 launch stereo_depth_node stereo_depth.launch.py \
    launch_hailo:=false \
    launch_imu:=false \
    launch_foxglove:=false
```

## Parameters

Main runtime parameters declared by `stereo_depth_node`:

- `depth_topic` (string, default `/depth/image`)
- `stereo_image_topic` (string, default `/stereo/image`)
- `calib_file` (string, default `/etc/hb_calib_params.json`)
- `config_file` (string, default `/etc/stereo_config.yaml`)
- `baseline_m` (double, default `0.14`)
- `gstreamer_pipeline` (string, default empty; auto-generated when empty)

Example with explicit files and baseline:

```bash
ros2 run stereo_depth_node stereo_depth_node --ros-args \
    -p calib_file:=/absolute/path/to/calib.json \
    -p config_file:=/absolute/path/to/default_config.yaml \
    -p baseline_m:=0.120
```

## Configuration Files

### Stereo config

Default example: `config/default_config.yaml`

Loaded with OpenCV FileStorage (YAML/JSON). Common fields include:

- `sgbm`, `sgbm_block_size`, `sgbm_p1`, `sgbm_p2`
- `num_disparities`
- `bm_block_size`, `bm_uniqueness_ratio`
- `isp_image_width`, `isp_image_height`
- `publish_pointcloud`
- visualization and tuning fields

### Calibration file

Runtime calibration is read as JSON by `StereoRgbCalParamHandler`.

Required keys:

- `camera_matrix_left`
- `camera_matrix_right`
- `distortion_coeffs_left`
- `distortion_coeffs_right`
- `extrinsics_left_T_right`
- `stereo_rectification_rotation_left`
- `stereo_rectification_rotation_right`
- `stereo_rectification_projection_left`
- `stereo_rectification_projection_right`

The calibration is scaled internally to match current per-eye capture resolution.

## Interfaces

### Custom message

`msg/StereoImage.msg`

- `std_msgs/Header header`
- `sensor_msgs/Image left_image`
- `sensor_msgs/Image depth_image`

This message keeps left image and depth image synchronized from the same source frame.

### Published topics

- `/stereo/image` (`stereo_depth_node/msg/StereoImage`)
- `/depth/image` (`sensor_msgs/msg/Image`, encoding `mono16`, depth in millimeters)
- `/left/image` (`sensor_msgs/msg/Image`, encoding `bgr8`)
- `/depth/image/colorized` (`sensor_msgs/msg/Image`, encoding `bgr8`, published when subscribed)
- `/stereo/pointcloud` (`sensor_msgs/msg/PointCloud2`, published when `publish_pointcloud=true`)
- `/stereo/pipeline_latency` (`diagnostic_msgs/msg/DiagnosticArray`)

## Capture and Processing Pipeline

1. Capture side-by-side UYVY frame from GStreamer.
2. Convert to BGR, split left/right, then grayscale.
3. Rectify both images using loaded calibration.
4. Compute disparity (SGBM or BM).
5. Convert disparity to depth using:

    `depth_m = (baseline_m * focal_length_px) / disparity_px`

6. Publish synchronized outputs and diagnostics.
7. Optionally publish point cloud.

## Docker Notes

Use the top-level [ros2_demo/docker](../../docker) directory for the combined public image and deployment flow.

The combined image bakes the contents of `config/combined_configs/` into `/opt/stereo_depth_node/configs`. On container startup, the top-level entrypoint copies any missing defaults into `/etc` and preserves any host-provided files that already exist.

## Troubleshooting

- If startup fails with calibration errors, verify `calib_file` exists and contains all required JSON keys.
- If no frames are published, validate camera device permissions and GStreamer pipeline compatibility.
- If depth looks incorrect, verify `baseline_m`, rectification matrices, and `num_disparities` tuning.
- If point cloud is missing, ensure `publish_pointcloud: true` in config.

## License

The package metadata declares `Apache-2.0` in `package.xml`. See the repository
root `LICENSE.txt` file for the full license text.
