# Based on: https://github.com/orbbec/OrbbecSDK_ROS2
ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# Create workspace
RUN mkdir -p ~/ros2_ws/src && \
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git src/OrbbecSDK_ROS2 && \
    # Install tools & dependencies
    apt-get update && apt install -y \
        libgflags-dev \
        nlohmann-json3-dev \
        libgoogle-glog-dev \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-publisher \
        ros-$ROS_DISTRO-camera-info-manager && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src -y -i && \
    # Install udev rules
    bash ./src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh && \
    rm -rf /var/lib/apt/lists/*

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

# Git action version
RUN echo $(cat /ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt

# Without this line Astra doesn't stop the camera on container shutdown. Default is SIGTERM.
STOPSIGNAL SIGINT
