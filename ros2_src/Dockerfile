# Use official ROS Humble full desktop image
FROM osrf/ros:humble-desktop-full

# Environment variable for ROS distro
ARG ROS_DISTRO=humble

# Install additional utilities and ROS packages you want
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    vim \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    xauth \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-robotiq-description \
    ros-${ROS_DISTRO}-urdf-tutorial \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-tf2-tools \
    gedit \
    evince \
    x11-apps \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user to avoid running as root
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
  && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup environment for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

# Copy entrypoint and bashrc files into image
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Fix permissions for scripts
RUN chmod +x /entrypoint.sh && chown $USERNAME:$USERNAME /home/${USERNAME}/.bashrc

# Switch to non-root user
USER $USERNAME

# Create and set workspace directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy local ROS packages into workspace
COPY ./ /home/$USERNAME/ros2_ws/src/

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build"

# Source workspace on container start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

