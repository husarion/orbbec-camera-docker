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

# Create health check package
RUN cd src/ && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp sensor_msgs && \
    sed -i '/find_package(sensor_msgs REQUIRED)/a \
            add_executable(healthcheck_node src/healthcheck.cpp)\n \
            ament_target_dependencies(healthcheck_node rclcpp sensor_msgs)\n \
            install(TARGETS healthcheck_node DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt

COPY ./healthcheck.cpp /ros2_ws/src/healthcheck_pkg/src/

# Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Save version
    echo $(cat /ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimalization
    rm -rf build log src

# Run healthcheck in background
RUN entrypoint_file=$(if [ -f "/ros_entrypoint.sh" ]; then echo "/ros_entrypoint.sh"; else echo "/vulcanexus_entrypoint.sh"; fi) && \
    sed -i '/test -f "\/ros2_ws\/install\/setup.bash" && source "\/ros2_ws\/install\/setup.bash"/a \
            ros2 run healthcheck_pkg healthcheck_node &' \
            $entrypoint_file

COPY ./healthcheck.sh /
HEALTHCHECK --interval=2s --timeout=1s --start-period=20s --retries=1 \
    CMD ["/healthcheck.sh"]

# Without this line Astra doesn't stop the camera on container shutdown. Default is SIGTERM.
STOPSIGNAL SIGINT
