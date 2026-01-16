# esp32-s3

To integrate your ESP32-S3 with the Raspberry Pi, we are going to use Micro-ROS.

This setup allows the ESP32 to act as a native ROS 2 node. The Raspberry Pi will run a "Micro-ROS Agent" that acts as a translator between the ESP32 (which speaks a lightweight protocol called XRCE-DDS) and your laptop (which speaks standard ROS 2).

**Why this is better**

The Raspberry Pi CPU is now free to handle the high-bandwidth camera data without the "stuttering" you felt before.

The ESP32 handles the motors in real-time. Even if the Pi's Linux kernel pauses for a second, the ESP32 keeps the PWM signals rock-solid.

## 1. Install the Micro-ROS Agent on the Raspberry Pi

The Agent is the "bridge" that lives on the Pi. It listens to the USB port where your ESP32 is plugged in and broadcasts the data to your network.

Run these commands on your Raspberry Pi terminal:

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

On your Laptop, open the Arduino IDE. You will need the micro_ros_arduino library (Install via Sketch -> Include Library -> Manage Libraries).

This code does three things:

Subscribes to /cmd_vel to drive the L298N.

Publishes IMU data from the MPU6050.

Synchronizes time with your ROS 2 clock.

[script](./motor_mpu.cpp)

### Core logic separation

Since you want a separate process on the ESP32-S3 to ensure low-latency motor control, we will use FreeRTOS Tasks.

By pinning the motor control and IMU reading to Core 1, we leave Core 0 entirely free to handle the Micro-ROS communication. This completely eliminates the "stuttering" you felt before because the high-speed networking won't interrupt the delicate timing of your PWM signals.

#### Why this works better

By using `xTaskCreatePinnedToCore`, we have created a "Hardware-timed loop" on Core 1.

Core 0 (Networking): Handles the WiFi/Serial overhead and Micro-ROS bookkeeping.

Core 1 (Physical): Dedicated to reading the MPU6050 and updating the L298N PWM. If Core 0 gets "stuck" briefly waiting for a packet, the motors on Core 1 will continue to hold their speed smoothly.

## 3. Final Integration on Raspberry Pi

Now that the ESP32 is ready, connect its USB/Native port to your Raspberry Pi and start the agent:

```sh
# On the Pi
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Check on your Laptop: Open a terminal and run ros2 topic list. You should see `/imu/data_raw` and `/cmd_vel` appearing as if they were coming from the raspberry


## 4. Essential: The "Permission" Fix

When you run the agent from a script, it might fail if it doesn't have permission to access /dev/ttyACM0. To prevent having to run sudo chmod every time you reboot, run this command once on your Raspberry Pi:

```sh
sudo usermod -a -G dialout $USER
```

(You will need to log out and log back in for this to take effect.)