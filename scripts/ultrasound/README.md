# ultrasound

This folder contains the scripts to use the sensor HC-SR04.
![image with 1 sensor](image_1.png)
![image with 2 sensors](image_3.png)

## scripts

There are 2 version:

- `ultrasound.py` for a single sensor
- `ultrasound_multiple` for multiple sensor (2 on the script)

## components

- HC-SR04 ultrasonic range sensor
- 1/8W 1% Axial Resistor 331F (330Ω)
- 1/8W 1% Axial Resistor 471F (470Ω)
- jump wires
- Breadboard

## schemas

![breadboard connections](image_2.png)
- HC-SR04 vcc - GPIO 5V
- HC-SR04 gnd - GPIO Ground
- HC-SR04 echo - Resistance 330Ω
- HC-SR04 trg - GPIO X
- GPIO Y - Resistance 330Ω - Resistance 470Ω
- Resistance 470Ω - gnd

For the sensor 1:
 - X = GPIO 23
 - Y = GPIO 24

For the sensor 2:
 - X = GPIO 20
 - Y = GPIO 21

## references

- [HC-SR04 Ultrasonic Range Sensor on the Raspberry Pi](https://thepihut.com/blogs/raspberry-pi-tutorials/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi)