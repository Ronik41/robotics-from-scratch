# Official ROS image, frozen to the manifest inspected on 2026-09-25.
FROM ros:jazzy-ros-base-noble@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8 AS robotics

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Use ROS's matching Gazebo vendor packages; do not mix OSRF repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-teleop-twist-keyboard \
    && dpkg-query -W > /opt/installed-packages.tsv \
    && rm -rf /var/lib/apt/lists/*

# Compile only the small ROS message package; no host installation.
COPY src/rover_interfaces /opt/rover_interfaces/src/rover_interfaces
RUN source /opt/ros/jazzy/setup.bash && cd /opt/rover_interfaces && colcon build --event-handlers console_direct+
# Compile the simulation motor boundary against the installed Harmonic SDK.
COPY src/rover_motor_driver /opt/rover_motor_driver/src
RUN source /opt/ros/jazzy/setup.bash && cmake -S /opt/rover_motor_driver/src -B /opt/rover_motor_driver/build -DCMAKE_INSTALL_PREFIX=/opt/rover_motor_driver && cmake --build /opt/rover_motor_driver/build -j2 && cmake --install /opt/rover_motor_driver/build
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/rover_motor_driver/lib
COPY scripts/ros-entrypoint.sh /ros_entrypoint.sh
WORKDIR /workspace
CMD ["bash"]

# Optional desktop stays inside Linux; macOS only needs an existing browser.
FROM robotics AS desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc novnc websockify openbox mesa-utils python3-pil x11-utils xterm \
    && dpkg-query -W > /opt/installed-packages.tsv \
    && rm -rf /var/lib/apt/lists/*
