# esp32-s3

This tutorial integrates an ESP32-S3 with a Raspberry Pi using Micro-ROS.

The ESP32 acts as a native ROS 2 node. The Raspberry Pi runs a Micro-ROS Agent that bridges the ESP32 (XRCE-DDS) to a laptop (standard ROS 2).

**Why this setup**

The Raspberry Pi stays free for high-bandwidth camera data without stuttering.

The ESP32 handles motor control in real time. Even if Linux pauses, PWM stays steady.

## 1. Install the Micro-ROS Agent on the Raspberry Pi

The agent is the bridge on the Pi. It listens to the USB port and broadcasts data on the network.

Run these commands on the Raspberry Pi terminal:

```sh
# 1. Source your ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# 2. Create a workspace for the agent
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git

# 3. Build the setup tool
cd ~/microros_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# 4. Create and build the agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## 2. The ESP32 "All-in-One" Firmware

On the laptop, open the Arduino IDE and install the `micro_ros_arduino` library (Sketch -> Include Library -> Manage Libraries).

This firmware does three things:

Subscribes to `/cmd_vel` to drive the L298N.

Publishes IMU data from the MPU6050.

Synchronizes time with the ROS 2 clock.

[script](./motor_mpu.cpp)

### Core logic separation

Use FreeRTOS tasks to keep motor control low latency.

Pin motor control and IMU reading to Core 1, and keep Core 0 free for Micro-ROS communication. This avoids stutter because networking does not interrupt PWM timing.

#### Why this works better

Using `xTaskCreatePinnedToCore` creates a hardware-timed loop on Core 1.

Core 0 (Networking): Handles WiFi/Serial overhead and Micro-ROS bookkeeping.

Core 1 (Physical): Reads the MPU6050 and updates L298N PWM. If Core 0 stalls, the motors on Core 1 keep steady speed.

## 3. Final Integration on Raspberry Pi

Once the ESP32 is ready, connect its USB/Native port to the Raspberry Pi and start the agent:

```sh
# On the Pi
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

On the laptop, run `ros2 topic list` and expect to see `/imu/data_raw` and `/cmd_vel` coming from the Raspberry Pi.


## 4. Essential: The "Permission" Fix

When the agent runs from a script, it can fail if it lacks permission to access `/dev/ttyACM0`. Fix this once so `sudo chmod` is not needed after every reboot:

```sh
sudo usermod -a -G dialout $USER
```

(Log out and back in for this to take effect.)

**NOTE** If the agent fails to start: check that the ESP32 is on `/dev/ttyACM0` and run `ls /dev/ttyACM*` to confirm.

## 5. Force restart ESP32

Install `esptool` python package

```sh
python3 -m pip install --user esptool --break-system-packages
```

Then you can restart the board using the command

```sh
esptool --port /dev/ttyACM0 --after hard_reset chip-id
```

It is integrated on the launch before start the agent.

**NOTE** Even if the agent connects on ros2 launch, you might need to restart the ESP32 to properly create the session between micro_ros on the raspberry and micro_ros_agent on the ESP32
