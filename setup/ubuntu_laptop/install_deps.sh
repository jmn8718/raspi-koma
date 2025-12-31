#!/bin/bash

set -e

# Install Dependencies
echo "Installing project dependencies..."
sudo apt install -y \
    python3-colcon-common-extensions \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui

