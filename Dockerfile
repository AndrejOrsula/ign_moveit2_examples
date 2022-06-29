ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}-ros-base

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Define working directory
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_INSTALL_DIR=${WS_DIR}/install
WORKDIR ${WS_DIR}

### Install Gazebo
ARG IGNITION_VERSION=fortress
ENV IGNITION_VERSION=${IGNITION_VERSION}
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ignition-${IGNITION_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Copy over ign_moveit2_examples
COPY ./ ${WS_SRC_DIR}/ign_moveit2_examples/

### Import and install dependencies, then build ign_moveit2_examples
WORKDIR ${WS_DIR}
RUN vcs import ${WS_SRC_DIR} < ${WS_SRC_DIR}/ign_moveit2_examples/ign_moveit2_examples.repos && \
    rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths ${WS_SRC_DIR} && \
    rm -rf /var/lib/apt/lists/* && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"

### Add workspace to the ROS entrypoint
### Source ROS entrypoint inside `~/.bashrc` to enable autocompletion
RUN sed -i '$i source "${WS_INSTALL_DIR}\/local_setup.bash"' /ros_entrypoint.sh && \
    sed -i '$a source /ros_entrypoint.sh' ~/.bashrc
