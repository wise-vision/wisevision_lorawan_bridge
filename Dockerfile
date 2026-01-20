ARG ROS_DISTRO=jazzy
FROM wisevision/ros_with_wisevision_msgs_wisevision_core_and_grpc:${ROS_DISTRO}

ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /root/wisevision_lorawan_bridge_ws

COPY . /root/wisevision_lorawan_bridge_ws/src/wisevision_lorawan_bridge

ENV GRPC_INSTALL_DIR=/root/grpc_install_dir
ENV PATH=$GRPC_INSTALL_DIR/bin:$PATH
ENV LD_LIBRARY_PATH=$GRPC_INSTALL_DIR/lib:/usr/local/lib:$LD_LIBRARY_PATH

RUN apt-get update && \
    if [ "${INSTALL_RMW_ZENOH}" = "1" ]; then apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-zenoh-cpp; fi && \
    rosdep fix-permissions && \
    rosdep update --include-eol-distros --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=""

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -n \"$RMW_IMPLEMENTATION\" ]; then export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; fi && \
    source /root/wisevision_ws/install/setup.bash && \
    colcon build"
