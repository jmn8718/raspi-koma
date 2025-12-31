#!/bin/bash

set -e

echo "Starting Raspi-Koma Laptop Setup..."

# Update System
echo "Updating system..."
sudo apt update && sudo apt upgrade -y

# Add ROS 2 Repository
echo "Configuring ROS 2 Jazzy repositories..."
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS 2 Jazzy (Desktop)
echo "Installing ROS 2 Jazzy Desktop..."
sudo apt install -y ros-jazzy-desktop
