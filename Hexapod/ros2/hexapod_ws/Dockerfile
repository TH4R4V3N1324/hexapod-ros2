FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    python3-colcon-common-extensions \
    python3-rosdep \
    terminator \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

WORKDIR /hexapod_ws
COPY src ./src

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /hexapod_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]