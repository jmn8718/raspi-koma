# raspi-koma

This project uses a regular RC Car toy as a base. I have scrapped all the parts and only use the chassis and the 2 motors.

As the car uses a DC motor for steering, it limits the turning functionality unless it is replaced with a *stepper motor*.

## Car dimensions

### Frame

Width: 22cm
Lenght: 43cm
Height: 7cm

### Tyres

Width: 3cm
Diameter: 7cm
Center from Front Tires: 12cm from the front
Center from Back Tires: 7cm from the back
Center of tire: 1cm over bottom of the car

### Ultrasonic sensor

Position From center: 4.5cm
Rotation: 25 degrees

## Camera

X: 7cm behind the front
Y: center of the vehicle (width)
Z: 7cm over the bottom

## Components

- Raspberry Pi 3 B+
- DC motor for linear movement
- DC motor for steering
- 2 HC-SR04 ultrasonic range sensor
- Raspberry Pi Camera v2 IMX219
- MPU-6050 (GY-521) 6-axis Motion

## Camera

It uses the camera to provide the car with "eyes".

You can test the camera with ros2

```sh
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p image_size:="[320,240]" \
  -p pixel_format:="YUYV"
```

*WARNING* depending of your ubuntu version and your camera, you might need to install the camera and configure it correctly.

## Ultrasonic sensors

With a 4.5 cm gap, if you kept them parallel ($0^\circ$), you would have a narrow "tunnel vision" and might miss a thin obstacle (like a chair leg) that passes right between the sensors or hits the side of the car.

By angling them at $25^\circ$:
- *The Overlap*: The sensors will still have a slight overlap in the far field (more than 1 meter away), meaning you won't have a blind spot in the distance.
- *The Corners*: As the car approaches a wall at an angle, the "outside" sensor will pick it up much earlier, giving your navigation code time to react before the chassis hits the obstacle.
- *The Width*: You effectively widen your detection "aperture" to cover roughly the full width of your car plus a safety margin on the sides.

## MPU

The MPU-6050 (GY-521) is a 6-axis Motion Tracking device that combines a 3-axis gyroscope, a 3-axis accelerometer and temperature sensor.


## Motors

### control 

Once the ros2 package is running, you can control the motors manually with

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
When you press 'i', the drive wheels should spin.

When you press 'j' or 'l', the front wheels should steer.

