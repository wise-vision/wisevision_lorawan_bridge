FROM wisevision/ros_with_wisevision_msgs_wisevision_core_and_grpc:humble

WORKDIR /root/wisevision_lorawan_bridge_ws

COPY . /root/wisevision_lorawan_bridge_ws/src/wisevision_lorawan_bridge

ENV GRPC_INSTALL_DIR=/root/grpc_install_dir
ENV PATH=$GRPC_INSTALL_DIR/bin:$PATH
ENV LD_LIBRARY_PATH=$GRPC_INSTALL_DIR/lib:/usr/local/lib:$LD_LIBRARY_PATH

RUN apt-get update && \
    rosdep fix-permissions && \
    rosdep update --include-eol-distros --rosdistro humble 

SHELL ["/bin/bash", "-c"]

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/wisevision_ws/install/setup.bash && \
    colcon build"

