FROM ros:humble-ros-base-jammy AS base

SHELL ["/bin/bash", "-c"]

RUN apt-get update

RUN apt-get install -y \
    software-properties-common \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-core \
    ros-humble-ament-package \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joy \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-generator-c \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-realsense2-camera \
    ros-humble-depth-image-proc \
    ros-humble-rplidar-ros \
    ros-humble-topic-tools \
    ros-humble-tf2-ros \
    libportaudio2 \
    x11-apps \
    libsm6 \
    ffmpeg

RUN python3 -m pip install --upgrade pip

ENV ROS_DISTRO=humble \
    ROS_ROOT=/opt/ros/${ROS_DISTRO} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

RUN mkdir /app
WORKDIR /app

RUN mkdir -p /app/limx_sdk
COPY . /app/limx_sdk

WORKDIR /app/limx_sdk

RUN rosdep install -y --ignore-src --from-paths . -r || true
# Install Python dependencies from pyproject.toml (numpy<2 for cv_bridge compatibility)
RUN pip install -e . --force-reinstall --ignore-installed --no-cache-dir
RUN source /opt/ros/humble/setup.bash && colcon build

RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS environment' >> /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /app/limx_sdk/install/setup.bash' >> /entrypoint.sh && \
    echo 'source /app/limx_sdk/ros2-bridger/aarch64/humble/install/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# If no arguments provided, run default command' >> /entrypoint.sh && \
    echo 'if [ $# -eq 0 ]; then' >> /entrypoint.sh && \
    echo '    # Launch mrosbridger in background' >> /entrypoint.sh && \
    echo '    ros2 launch mrosbridger mrosbridger.launch.py &' >> /entrypoint.sh && \
    echo '    # Launch sensor launch as main process' >> /entrypoint.sh && \
    echo '    exec ros2 launch tron_sdk sensor_launch.py' >> /entrypoint.sh && \
    echo 'else' >> /entrypoint.sh && \
    echo '    exec "$@"' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null
RUN apt update && apt install zenoh-bridge-ros2dds -y

ENTRYPOINT ["/entrypoint.sh"]
