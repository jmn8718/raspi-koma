# External Ubuntu Laptop Setup Guide

This guide explains how to configure an external Ubuntu (24.04) laptop to communicate with the Raspi-Koma robot.

## Prerequisites
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **Network**: The laptop and Robot must be on the **same Wi-Fi network**.

## Quick Start (Automated)

We have provided a script to automate the installation of ROS 2 Jazzy and all required dependencies.

1.  Clone this repository to your laptop.
2.  Navigate to the setup folder:
    ```bash
    cd raspi-koma/setup/ubuntu_laptop
    ```
3.  Make the scripts executable:
    ```bash
    chmod +x install_ros.sh install_deps.sh build_ws.sh
    ```
4.  Run the setup scripts in order:

    **Step 1: Install ROS 2**
    ```bash
    ./install_ros.sh
    ```

    **Step 2: Install Dependencies**
    ```bash
    ./install_deps.sh
    ```

    **Step 3: Build Workspace**
    ```bash
    ./build_ws.sh
    ```

5.  Source the workspace:
    ```bash
    source ../../view_ws/install/setup.bash
    ```

---

## Manual Installation Steps

If the script fails or you prefer manual control:

### 1. Install ROS 2 Jazzy
Follow the official [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

### 2. Install Project Dependencies
These packages are required for visualization and control:

```bash
sudo apt install -y \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui
```

### 3. Build the Workspace
The `view_ws` folder contains the launch files and configuration for the laptop side.

```bash
cd raspi-koma/view_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Application

### 1. Start the Robot
SSH into your robot and launch the controller:
```bash
# On Robot
ros2 launch car_controller car_launch.py
```

### 2. Start Visualization (Laptop)
On your laptop, run the visualization launch file. This starts RViz, the Joystick node, and the EKF filter.

```bash
# On Laptop
ros2 launch car_description rviz_launch.py
```

### 3. Verify Connection
open a new terminal on your laptop and check if you can see the topics:
```bash
ros2 topic list
```
You should see topics like `/camera/image_raw`, `/imu/data`, etc.

## Troubleshooting

### "Topics not found"
If you cannot see the ROS topics:
1.  **Check IP Addresses**: Ensure both devices have unique IPs on the same subnet.
2.  **Multicast**: Ensure your network allows multicast.
3.  **ROS_DOMAIN_ID**: If you changed this on the robot, ensure you export the same ID on your laptop:
    ```bash
    export ROS_DOMAIN_ID=<your_id>
    ```

### Joystick not working
- Ensure your joystick is connected (USB/Bluetooth).
- Check if it is detected: `ls /dev/input/js*`.
- If it is on `js1` instead of `js0`, edit `view_ws/src/car_description/launch/rviz_launch.py`.
