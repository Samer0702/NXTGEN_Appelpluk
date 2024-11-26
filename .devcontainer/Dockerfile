FROM moveit/moveit2:humble-release
ARG DISTRO=humble

# Make sure DDS uses only localhost
ENV ROS_LOCALHOST_ONLY=1
ENV DISPLAY="unix:1"

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

RUN usermod -a -G video ${USERNAME}

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && rm -rf /var/lib/apt/lists/*

# Install necessary packages
RUN apt-get update && apt-get install -y \
    ros-${DISTRO}-dynamixel-sdk \
    ros-${DISTRO}-dynamixel-workbench-toolbox \
    ros-${DISTRO}-xacro \
    ros-${DISTRO}-joint-state-publisher \
    ros-${DISTRO}-joint-state-publisher-gui \
    ros-${DISTRO}-ros-gz \
    ros-${DISTRO}-ros2-control \
    ros-${DISTRO}-ros2-controllers \
    ros-${DISTRO}-ign-ros2-control \
    ros-${DISTRO}-robot-localization \
    ros-${DISTRO}-navigation2 \
    ros-${DISTRO}-tf-transformations \
    ros-${DISTRO}-gazebo-ros-pkgs \
    ros-${DISTRO}-usb-cam \
    ros-${DISTRO}-ur \
    ros-${DISTRO}-moveit \
    ignition-fortress \
    python3-pip \
    libgl1-mesa-dri libgl1-mesa-glx libegl1-mesa mesa-utils \
    gazebo \
    net-tools \
    python3-numpy \
    libboost-python-dev \
    libopencv-dev \
    python3-deprecated \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install opencv-python


# Source ROS in user's bashrc
RUN echo "source /opt/ros/${DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

# Switch to non-root user
USER ${USERNAME}

CMD ["bash"]