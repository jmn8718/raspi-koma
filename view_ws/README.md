# View workspace

View workspace is a ROS 2 workspace that contains the URDF description and visualization tools for the raspikoma robot.

## Install libraries 

```sh
## dependencies for robot description and visualization
sudo apt install ros-jazzy-imu-filter-madgwick ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher
## dependencies for teleop with a joystick / controller
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
## dependencies for robot localization
sudo apt install ros-jazzy-robot-localization
```

## Build and source workspace

```sh
cd ~/view_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch RViz

```sh
cd ~/view_ws
ros2 launch car_description rviz_launch.py
```
