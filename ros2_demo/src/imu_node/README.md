# imu_node

A ROS 2 (Humble, `ament_python`) node that bridges IMU samples published as
JSON over [Zenoh](https://zenoh.io/) into ROS 2 topics.

For each Zenoh sample received on the configured key, the node publishes:

- `sensor_msgs/Imu` on `imu_topic` (default `/imu/data`)
- `std_msgs/String` containing the raw JSON payload on `raw_json_topic`
  (default `/imu/raw_json`)

The publishers use sensor-data style QoS: best-effort, volatile,
keep-last(depth=1).

## Expected JSON schema

The upstream stereo device publishes `ImuSample` records on Zenoh key
`hb/stereo/imu/stream`:

```json
{
    "accel":        {"x": 0.122, "y": 0.104, "z": 9.809},
    "gyro":         {"x": 0.0161, "y": 0.0046, "z": -0.00053},
    "timestamp_us": 521941081
}
```

`accel` (m/s^2) maps to `Imu.linear_acceleration`, `gyro` (rad/s) to
`Imu.angular_velocity`, and `timestamp_us` to the message header stamp.

Orientation behavior:

- If `estimate_orientation_from_accel=true` and accel data is present, the
  node derives tilt orientation from the gravity vector.
- If orientation is estimated, `orientation_covariance` is all zeros
  (measurement present, covariance unknown).
- If orientation is not estimated, orientation is zeroed and
  `orientation_covariance[0] = -1.0` (no orientation estimate).

`frame_id` is taken from `config/imu_node_config.json` (or the file provided
via the launch argument).

## Configuration

`config/imu_node_config.json` controls Zenoh session parameters, frame
metadata, orientation-estimation behavior, and default angular/linear
covariance values. The launch file passes its path through the `config_file`
parameter.

Key fields:

- `zenoh_key`: Zenoh key expression to subscribe to.
- `zenoh_endpoints`: Optional list of router/peer endpoints.
- `zenoh_mode`: `peer` or `client`.
- `frame_id`: `frame_id` stamped on published `sensor_msgs/Imu`.
- `estimate_orientation_from_accel`: Enable accel-based tilt orientation
  estimation.
- `gravity_reference_axis`: Body-frame axis considered "up" when level.
- `angular_velocity_covariance`: 3x3 row-major covariance for angular
  velocity.
- `linear_acceleration_covariance`: 3x3 row-major covariance for linear
  acceleration.

## Launch arguments

`launch/imu.launch.py` currently accepts:

- `imu_topic` (default `/imu/data`)
- `raw_json_topic` (default `/imu/raw_json`)
- `config_file` (default `/etc/imu_node_config.json`)

Example:

```bash
ros2 launch imu_node imu.launch.py \
  imu_topic:=/robot/imu \
  raw_json_topic:=/robot/imu_raw \
  config_file:=/etc/imu_node_config.json
```

## Dependencies (rosdep + pip)

`eclipse-zenoh` is a pip dependency. This repository includes a local rosdep
mapping in `rosdep.yaml` for key `python3-eclipse-zenoh`.

Before running `rosdep install`, register the local rosdep source:

```bash
rosdep update
rosdep install --from-paths src --rosdep-sources-file rosdep.yaml --ignore-src -r -y
```

## Build & run

```bash
# In a ROS 2 Humble workspace:
colcon build --packages-select imu_node
source install/setup.bash
ros2 launch imu_node imu.launch.py
```

## Tests

Unit tests cover both configuration loading and JSON-to-Imu conversion logic,
including malformed payload handling and tilt-quaternion edge cases.

```bash
colcon test --packages-select imu_node
colcon test-result --verbose
```
