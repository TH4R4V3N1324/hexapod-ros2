ARG ROS_IMAGE=ros:jazzy-ros-base
ARG INSTALL_DEV_TOOLS=false

FROM ${ROS_IMAGE}

ARG INSTALL_DEV_TOOLS
ENV INSTALL_DEV_TOOLS=${INSTALL_DEV_TOOLS}

SHELL ["/bin/bash", "-c"]

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-position-controllers \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-foxglove-msgs \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_DEV_TOOLS" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joy \
    terminator \
    && rm -rf /var/lib/apt/lists/*; \
fi

# Initialize rosdep and update
RUN rosdep init || true && rosdep update

# Create workspace and copy source code
WORKDIR /hexapod_ws
COPY src ./src

# Install dependencies.
# Refresh apt metadata immediately before rosdep to avoid stale package URLs (404s).
RUN source /opt/ros/jazzy/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the setup files in .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /hexapod_ws/install/setup.bash" >> /root/.bashrc

# Auto start
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && \
    source /hexapod_ws/install/setup.bash && \
    ros2 launch hexapod_bringup hexapod_fake.launch.py"]