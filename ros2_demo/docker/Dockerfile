# syntax=docker/dockerfile:1.6
# ---------------------------------------------------------------------------
# Public ROS 2 stereo demo image.
#
# All three ROS packages (stereo_depth_node, hailo_detection_node, imu_node)
# are shipped in this repo under src/ as code-only snapshots — CI builds
# strictly from committed code and never clones any external repo.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Stage 1: Build hailort from source (Hailo-8 runtime + Python bindings)
# ---------------------------------------------------------------------------
FROM ros:humble AS hailort-build

RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf automake cmake ninja-build pkg-config \
    git build-essential \
    libgnutls28-dev zlib1g-dev \
    curl ca-certificates libcurl4-openssl-dev libssl-dev \
    libpython3-dev python3-pip python3-dev python3-setuptools \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    rsync nasm libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /hailort && \
    git clone https://github.com/hailo-ai/hailort.git /hailort/sources && \
    cd /hailort/sources && git checkout v4.20.1

RUN cd /hailort/sources && \
    cmake -S. -Bbuild \
    -DCMAKE_BUILD_TYPE=Release \
    -DHAILO_BUILD_EXAMPLES=1 \
    -DHAILO_BUILD_GSTREAMER=1 && \
    cmake --build build --config Release --target install -- -j$(nproc)

RUN cd /hailort/sources/hailort/libhailort/bindings/python/platform && \
    python3 setup.py bdist_wheel

# ---------------------------------------------------------------------------
# Stage 2: Build the ROS 2 workspace from the in-tree src/ snapshots
# ---------------------------------------------------------------------------
FROM ros:humble AS ros-build

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config \
    libgstreamer1.0-0 \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    libopencv-dev \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-msgs \
    ros-humble-vision-opencv \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY src/ /ros2_ws/src/

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install \
        --packages-select stereo_depth_node hailo_detection_node imu_node

# ---------------------------------------------------------------------------
# Stage 3: Minimal runtime image
# ---------------------------------------------------------------------------
FROM ros:humble AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-msgs \
    ros-humble-vision-opencv \
    ros-humble-rosidl-default-runtime \
    ros-humble-foxglove-bridge \
    libopencv-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY --from=hailort-build /usr/local/lib/libhailort* /usr/local/lib/
COPY --from=hailort-build /hailort/sources/hailort/libhailort/bindings/python/platform/dist/*.whl /tmp/
RUN python3 -m pip install --no-cache-dir /tmp/*.whl && rm /tmp/*.whl

RUN python3 -m pip install --no-cache-dir eclipse-zenoh==1.8.0

RUN ldconfig

COPY --from=ros-build /ros2_ws/install /ros2_ws/install

# Baked-in default configs. The entrypoint copies any missing entries into /etc
# on first start so no host-side setup is required.
COPY config/combined_configs /opt/stereo_depth_node/configs

# Baked-in Hailo networks, installed to /networks the same way. The detection
# node loads <model_path>/<model_name>/<model_name>.hef.
COPY networks /opt/stereo_depth_node/networks

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENV ROS_DISTRO=humble
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "stereo_depth_node", "stereo_depth.launch.py"]
