ARG ROS_IMAGE=ros:humble-ros-base
ARG INSTALL_DEV_TOOLS=false

FROM ${ROS_IMAGE}

ARG INSTALL_DEV_TOOLS
ENV INSTALL_DEV_TOOLS=${INSTALL_DEV_TOOLS}

SHELL ["/bin/bash", "-c"]

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_DEV_TOOLS" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \   
    ros-humble-joint-state-publisher-gui \
    ros-humble-position-controllers \
    ros-humble-joy \
    terminator \
    && rm -rf /var/lib/apt/lists/*; \
fi

# Initialize rosdep and update
RUN rosdep init || true
RUN rosdep update

# Create workspace and copy source code
WORKDIR /hexapod_ws
COPY src ./src
RUN git clone https://github.com/foxglove/foxglove-sdk.git /tmp/foxglove-sdk --depth 1 && \
    cp -r /tmp/foxglove-sdk/ros/src/foxglove_bridge src/foxglove_bridge && \
    rm -rf /tmp/foxglove-sdk

# Install dependencies.
# Refresh apt metadata immediately before rosdep to avoid stale package URLs (404s).
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the setup files in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /hexapod_ws/install/setup.bash" >> /root/.bashrc

# Auto start
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /hexapod_ws/install/setup.bash && ros2 launch hexapod_bringup hexapod_fake.launch.py"]