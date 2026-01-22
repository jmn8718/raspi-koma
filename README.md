# raspi-koma

This project uses a standard RC car chassis with the original two DC motors.

Because the steering is a DC motor, turning precision is limited unless it is replaced with a stepper motor.

## Hardware

### Boards
- Raspberry Pi 3 B+
- ESP32-S3 (QFN56, revision v0.2)

### Car dimensions
Frame:
- Width: 22 cm
- Length: 43 cm
- Height: 7 cm

Tires:
- Width: 3 cm
- Diameter: 7 cm
- Center from front tires: 12 cm from the front
- Center from back tires: 7 cm from the back
- Center of tire: 1 cm above the bottom of the car

### Ultrasonic sensor
- Position from center: 4.5 cm
- Rotation: 25 degrees

### Camera mount
- X: 7 cm behind the front
- Y: centered on the vehicle width
- Z: 7 cm above the bottom

### Components
- DC motor for linear movement
- DC motor for steering
- 2x HC-SR04 ultrasonic range sensor
- Raspberry Pi Camera v2 IMX219
- MPU-6050 (GY-521) 6-axis motion sensor

## Camera

The camera provides the vehicle with a live ROS 2 stream.

Test the camera with ROS 2:

```sh
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p image_size:="[320,240]" \
  -p pixel_format:="YUYV"
```

*WARNING* Depending on your Ubuntu version and camera, you may need to install and configure the camera driver.

## Ultrasonic sensors

With a 4.5 cm gap, if they were kept parallel (0 degrees), the car would have a narrow "tunnel vision" and could miss thin obstacles (like a chair leg) between the sensors or near the side of the car.

By angling them at 25 degrees:
- The overlap: The sensors still overlap in the far field (beyond 1 meter), so there is no distant blind spot.
- The corners: Approaching a wall at an angle, the outside sensor detects it earlier and gives navigation time to react.
- The width: The detection aperture covers roughly the full width of the car plus a safety margin.

## MPU

The MPU-6050 (GY-521) is a 6-axis motion sensor with a 3-axis gyroscope, 3-axis accelerometer, and temperature sensor.

## Motors

### Control

Once the ROS 2 package is running, control the motors manually with:

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When you press `i`, the drive wheels should spin.

When you press `j` or `l`, the front wheels should steer.
