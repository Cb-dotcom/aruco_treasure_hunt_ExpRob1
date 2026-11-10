# ArUco Treasure Hunt

Experimental Robotics Laboratory, Assignment 1 (UniGe, 2026/27).

A differential-drive robot follows four ArUco markers. It reaches each marker with image-based
visual servoing, publishes the marker image on `/marker_image`, executes the marker's instruction
with a closed-loop rotation on odometry, and stops at marker 4. The same code runs in Gazebo and
on the ROSbot; only the parameter file changes.

## Requirements

- Ubuntu 22.04 + ROS 2 Humble, or Ubuntu 24.04 + ROS 2 Jazzy
- Gazebo Harmonic
- The course package [erl1](https://github.com/CarmineD8/erl1)

```bash
# Humble (Gazebo Harmonic from packages.osrfoundation.org)
sudo apt install ros-humble-ros-gzharmonic ros-humble-robot-localization ros-humble-xacro \
  ros-humble-cv-bridge ros-humble-image-transport-plugins ros-humble-rqt-image-view libopencv-dev
# Jazzy
sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-localization ros-jazzy-xacro \
  ros-jazzy-cv-bridge ros-jazzy-image-transport-plugins ros-jazzy-rqt-image-view libopencv-dev
```

## Build

```bash
mkdir -p ~/exprob_ws/src && cd ~/exprob_ws/src
git clone <this repository> aruco_treasure_hunt
git clone https://github.com/CarmineD8/erl1.git
mkdir -p ~/gazebo_models && cp -r erl1/models/. ~/gazebo_models/   # aruco_box_* models
cd ~/exprob_ws && colcon build --symlink-install && source install/setup.bash
```

## Team

| Name | GitHub |
|---|---|
| Charbel El Beaini | [@Cb-dotcom](https://github.com/Cb-dotcom) |
| | |
| | |
| | |
