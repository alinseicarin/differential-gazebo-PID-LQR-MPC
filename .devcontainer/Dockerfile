# Start from the official ROS 2 Desktop image
FROM osrf/ros:humble-desktop

# Define our default user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Install Gazebo, pip, and basic development tools
RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Grant the user passwordless sudo privileges
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME
WORKDIR /home/ws

# Source the ROS 2 setup script automatically on every terminal launch
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc