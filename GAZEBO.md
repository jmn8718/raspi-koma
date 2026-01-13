# Gazebo Simulator

## Installation

1. Install via ROS 2 Jazzy Meta-package
Instead of the standalone gz-harmonic, use the ROS 2 integration package which will pull in all the necessary Gazebo Harmonic dependencies automatically:

```sh
sudo apt update
sudo apt install ros-jazzy-ros-gz -y
```

2. Verify the Installation
Once the installation finishes, you need to source your ROS environment to see the gz commands:

```sh
source /opt/ros/jazzy/setup.bash
gz sim -v 4 shapes.sdf
```

If gz sim isn't found, try installing the tools vendor package specifically: `sudo apt install ros-jazzy-gz-tools-vendor`

## Usage

1. Launch a Test World
Start a standard empty world to make sure the physics engine and GUI are communicating:

```sh
# Source your ROS 2 environment first
source /opt/ros/jazzy/setup.bash

# Launch an empty world with physics enabled
gz sim -v 4 empty.sdf
```