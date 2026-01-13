# ROS2

This project uses ros2 on ubuntu server. Tested with 24.04

## Requirements

You have to install ROS 2 on the Raspberry Pi, so you need an Ubuntu server running on it.

> **Note:** This guide is for setting up the **Robot (Raspberry Pi)**. If you are setting up your external computer for visualization and control, please see [SETUP_LAPTOP.md](../SETUP_LAPTOP.md).

You need to install some dependencies:

```sh
sudo apt install gpiozero python3-yaml python3-gpiozero python3-colcon-common-extensions -y
```

You might have to install *gpiozero* manually if your system does not include it.

## Configuration

We have to setup a domain ID so other devices can identify the domain. Ensure your laptop uses the same ID.

```sh
export ROS_DOMAIN_ID=87
```

### Build

Build the project and generate a ros package

```sh
cd robot_ws

# Delete old build/install folders to ensure a clean start
rm -rf build/ install/ log/ 

colcon build --symlink-install
# colcon build --symlink-install --packages-select car_controller

source install/setup.bash
```

### Run

In order to run the package, execute:

```sh
ros2 launch car_controller car_launch.py
```

### Makefile

`Makefile` command:
- *build*
- *clean*
- *run*

### bash

A *ros.bash* file is provided so you can just `source ros/ros.bash` and it will set *ROS_DOMAIN_ID* and source *setup.bash* for the package to be available on the terminal.

Other option is add this 2 options to you `.bashrc` so it loads everytime your terminal starts.

```sh
echo "export ROS_DOMAIN_ID=87" >> ~/.bashrc
echo "source ${PWD}/install/setup.bash" >> ~/.bashrc
```

## TO-DOs

[x] Manual "Dumb" RC Car
[ ] Follow a line
[ ] Simple "Obstacle Avoidance"