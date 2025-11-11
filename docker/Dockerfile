# Base Humble Image
FROM osrf/ros:humble-desktop

# No Interactive Prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3\* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-map-server \
    ros-humble-rmw-cyclonedds-cpp \
    xdg-utils vim-tiny nano \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations \
    python3-transforms3d \
 && rm -rf /var/lib/apt/lists/*

# Updates Motion Model YAML for AMCL
RUN sed -i 's/^ *robot_model_type: "differential"/robot_model_type: "nav2_amcl::DifferentialMotionModel"/' \
    /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml || true

# Auto-source ROS in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# --- Add tmux for terminal automation ---
RUN apt-get update && apt-get install -y tmux

# --- Copy tmux config into the container ---
COPY .tmux.conf /root/.tmux.conf

# --- Copy tmux scripts into the container ---
COPY scripts/slam_tmux.sh /root/scripts/slam_tmux.sh
COPY scripts/nav_tmux.sh /root/scripts/nav_tmux.sh
COPY scripts/survey_tmux.sh /root/scripts/survey_tmux.sh

# --- Make them executable ---
RUN chmod +x /root/scripts/*.sh

# Sets default command to bash
CMD ["bash"]