# ros2_demo — ROS 2 Stereo Camera Demo

A public, buildable demo of the Hellbender stereo camera ROS 2 stack. See [this page](https://github.com/HellbenderInc/hellbender-stereo-camera/releases) for the latest docker image and installation instructions. The container bundles three nodes:

- `stereo_depth_node` (C++): Stereo capture, SGBM disparity, and rectified stream publication.
- `hailo_detection_node` (Python): Person detection on Hailo-8 over the color stream.
- `imu_node` (Python): Consumes IMU JSON over Zenoh and republishes `sensor_msgs/Imu`.

## Contents

```text
ros2_demo/
├── docker/
│   ├── Dockerfile           # Multi-stage: hailort → colcon → runtime
│   ├── docker-compose.yml
│   └── ros_entrypoint.sh
├── src/
│   ├── stereo_depth_node/    # code-only snapshot
│   ├── hailo_detection_node/ # code-only snapshot
│   └── imu_node/             # code-only snapshot
└── SNAPSHOT.md               # commit SHAs of each upstream repo
```

Everything under `src/` is a **committed, code-only snapshot** of the
upstream repository at the SHA recorded in `SNAPSHOT.md`.

## Build locally

```bash
cd ros2_demo
docker buildx build --platform linux/arm64 --target runtime -t hb-stereo-ros2-demo:local -f docker/Dockerfile .
```

## Installation

See the product documentation for loading docker containers.
