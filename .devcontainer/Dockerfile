# Reproducible ROS 2 Humble development image for building and running the
# controller, Gazebo Classic, RViz, and robot_localization dependencies.
# The desktop image already provides the ROS client libraries and GUI tools.
FROM osrf/ros:humble-desktop

# Match the container account IDs to the host account by default. This prevents
# files created through the bind-mounted workspace from becoming root-owned.
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root interactive development account and its matching group.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Install integration packages not guaranteed by the base desktop image.
# gazebo-ros-pkgs provides the ROS system/model plugins used by the URDF and
# launch file; pip supports any later Python analysis or tooling dependencies.
RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    python3-yaml \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-osqp-vendor \
    && rm -rf /var/lib/apt/lists/*

# Passwordless sudo is convenient for rosdep and package installation inside a
# disposable development container. It should not be copied to production images.
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# All interactive commands and generated workspace files use the regular user.
USER $USERNAME

# devcontainer.json bind-mounts the host repository at this location.
WORKDIR /home/ws

# Make base ROS 2 commands available in each interactive Bash terminal. The
# workspace overlay still needs `source /home/ws/install/setup.bash` after build.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
