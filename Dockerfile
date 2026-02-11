ARG ROS_DISTRO=jazzy
ARG INSTALL_RMW_ZENOH=0
FROM wisevision/ros_with_wisevision_msgs_wisevision_core_and_grpc:${ROS_DISTRO}

# Re-declare build args after FROM so they are in scope for subsequent ENV/RUN
# (ARG scope is reset at each FROM; the earlier ARGs are only for the FROM line)
ARG ROS_DISTRO
ARG INSTALL_RMW_ZENOH
ENV ROS_DISTRO=${ROS_DISTRO}
ENV INSTALL_RMW_ZENOH=${INSTALL_RMW_ZENOH}

WORKDIR /root/wisevision_lorawan_bridge_ws

COPY . /root/wisevision_lorawan_bridge_ws/src/wisevision_lorawan_bridge

ENV GRPC_INSTALL_DIR=/root/grpc_install_dir
ENV PATH=$GRPC_INSTALL_DIR/bin:$PATH
ENV LD_LIBRARY_PATH=$GRPC_INSTALL_DIR/lib:/usr/local/lib:$LD_LIBRARY_PATH

RUN apt-get update && \
    if [ "${INSTALL_RMW_ZENOH}" = "1" ]; then \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-zenoh-cpp && \
      test -e "/opt/ros/${ROS_DISTRO}/lib/librmw_zenoh_cpp.so"; \
    fi && \
    rosdep fix-permissions && \
    rosdep update --include-eol-distros --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=""

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ -n \"$RMW_IMPLEMENTATION\" ]; then export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; fi && \
    source /root/wisevision_ws/install/setup.bash && \
    colcon build"
