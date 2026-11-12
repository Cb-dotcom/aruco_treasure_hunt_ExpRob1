# ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04): the professor's setup.
ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo \
      ros-jazzy-ros-gz \
      ros-jazzy-robot-localization \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-rqt-image-view \
      ros-jazzy-teleop-twist-keyboard \
      libopencv-dev \
      python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# The 24.04 image already has a uid 1000 user ("ubuntu"): rename it.
RUN usermod -l ${USERNAME} -d /home/${USERNAME} -m ubuntu && groupmod -n ${USERNAME} ubuntu \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
