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

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && rm -rf /var/lib/apt/lists/*

#add user to video group
RUN sudo usermod -a -G video ros

# Set up dependencies required for visual servoing
RUN apt-get update \
  && apt-get install -y libasio-dev \
  && apt-get install -y python3 \
  && apt-get install -y python3-pip \ 
  && pip3 install opencv-python \
  && apt install -y python3-numpy \
  && apt install -y libboost-python-dev \
  && apt install -y libopencv-dev \
  && apt-get install -y ros-${DISTRO}-usb-cam \
  && apt-get install -y ros-${DISTRO}-ur \
  && apt-get install -y ros-${DISTRO}-ros2-control \
  && apt-get install -y ros-${DISTRO}-ros2-controllers \
  && apt-get install -y net-tools \
  && rm -rf /var/lib/apt/lists/*  

RUN echo "source /opt/ros/${DISTRO}/setup.bash" >> /home/ros/.bashrc

USER ${USERNAME}

CMD ["bash"]

