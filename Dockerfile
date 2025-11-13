FROM ros:humble-ros-base-jammy

# environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# add any deps we want
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    vim \
    wget \
    curl \
    openssh-server \
    # ros deps
    ros-humble-foxglove-bridge \ 
    ros-${ROS_DISTRO}-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN mkdir /var/run/sshd
COPY ./sshd_override.conf /etc/ssh/sshd_config.d/override.conf
RUN echo 'root:thispasswordissecure' | chpasswd
EXPOSE 9095
EXPOSE 2267

# python deps
RUN pip install uv
COPY pyproject.toml uv.lock /workspace/
COPY uvsync.sh /workspace/uvsync.sh
RUN chmod +x /workspace/uvsync.sh
RUN /workspace/uvsync.sh

RUN rosdep update

# ros2 setup for bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

# ros2 setup for docker run (not bash)
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]

CMD ["bash"]