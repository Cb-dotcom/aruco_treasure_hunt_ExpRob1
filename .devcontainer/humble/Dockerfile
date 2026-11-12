# ROS 2 Humble + Gazebo Harmonic (Ubuntu 22.04). Humble ships with Gazebo Fortress by default;
# erl1 needs Harmonic, so ros-humble-ros-gzharmonic comes from the OSRF repository.
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release sudo \
 && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      ros-humble-robot-localization \
      ros-humble-robot-state-publisher \
      ros-humble-xacro \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-image-transport-plugins \
      ros-humble-rqt-image-view \
      ros-humble-teleop-twist-keyboard \
      libopencv-dev \
      python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
