FROM dorowu/ubuntu-desktop-lxde-vnc:focal

RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG ROS_DISTRO=galactic
ARG INSTALL_PACKAGE=desktop

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

#RUN sudo apt-get update -q && \
#    apt-get install ros-${ROS_DISTRO}-turtlesim && \
#    apt-get install ros-${ROS_DISTRO}-rqt* && \
#    apt-get install ros-${ROS_DISTRO}-ros2bag && \
#    apt-get install ros-${ROS_DISTRO}-rosbag2-storage-default-plugins

RUN gosu ubuntu rosdep update && \
    grep -F "source /opt/ros/${ROS_DISTRO}/setup.bash" /home/ubuntu/.bashrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

RUN sed -i "s#location ~ .*/(api/.*|websockify) {#location ~ .*/(api/.*|websockify|resize) {#" /etc/nginx/sites-enabled/default

ENV USER ubuntu