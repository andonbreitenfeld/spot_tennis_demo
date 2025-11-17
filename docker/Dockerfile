FROM ultralytics/ultralytics:latest-jetson-jetpack6

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DISTRO=${ROS_DISTRO}

# Install ROS2 Humble
RUN apt update \
    && apt install -y software-properties-common curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt install -y \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Fix setuptools
RUN pip install 'setuptools<70.0.0' 'numpy<2'

# Build yolo_ros
WORKDIR /colcon_ws/src
RUN git clone https://github.com/mgonzs13/yolov8_ros.git

WORKDIR /colcon_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
