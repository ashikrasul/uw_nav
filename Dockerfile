# ROS 2 Humble image for the nav stack (drivers, robot_localization, beacon_locator, foxglove_bridge, rosbag2).
# Multi-arch base: builds on the x86 PC and on an arm64 Pi/Jetson.
# Source is not copied in; docker/docker-compose.yml mounts src/ and utils/.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-serial \
        python3-colcon-common-extensions \
        python3-numpy \
        i2c-tools \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-diagnostic-updater \
        ros-humble-foxglove-bridge \
        ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Sensor drivers (versions tested on the host). The GPS/BNO055 pins match the reference
# submodules in utils/Adafruit_CircuitPython_*; keep them in sync.
RUN pip3 install --no-cache-dir \
        Adafruit-Blinka==9.2.0 \
        Jetson.GPIO==2.1.13 \
        adafruit-circuitpython-gps==4.0.0 \
        adafruit-circuitpython-bno055==5.4.22 \
        adafruit-extended-bus==1.0.2

# Run as the host user's UID: /dev:/dev also shares the host's /dev/shm, and Fast DDS sends
# data through shared-memory files there. Root-owned files are unusable by host ROS processes.
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g ${USER_GID} ros && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -G dialout -s /bin/bash ros && \
    mkdir -p /ros2_ws && chown ros:ros /ros2_ws

# Interactive shells (enter_console.sh) get ROS and the workspace sourced
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/ros/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /home/ros/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ros
WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "nav_bringup", "nav.launch.py"]
