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

## Ubuntu

On Ubuntu Server 24.04, the script to execute is `ultrasound_gpio.py`.


### Warning

If the script show the warning `packages/gpiozero/input_devices.py:852: PWMSoftwareFallback...`.

You have to install and Enable the pigpio Daemon

```sh
# Install the daemon and python library
sudo apt update
sudo apt install pigpio python3-pigpio

# Start the service and set it to run on every boot
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

If it fails to install pigpio on ubuntu, then you have to install manually *pigpio* if it is not found on the system.

1. Build and Install pigpio from Source

```sh
# Install build tools
sudo apt update
sudo apt install -y wget unzip build-essential

# Download and compile pigpio
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

2. Set up the Daemon as a Service

Since apt didn't handle the install, we have to manually create the "service" so the daemon starts automatically when the Pi boots.

2.1. Create the service file: `sudo nano /lib/systemd/system/pigpiod.service`.
2.2. Paste this exact text into the file:
```
[Unit]
Description=Daemon required to control GPIO pins via pigpio
[Service]
ExecStart=/usr/local/bin/pigpiod -l
ExecStop=/bin/systemctl kill -s SIGKILL pigpiod
Type=forking
[Install]
WantedBy=multi-user.target
```
2.3. Start the service:
```sh
sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

3. Verification

Check if the daemon is running correctly:

```sh
pigs hwver
```

If it returns a number (like 10444811), success! Your Pi is now using hardware-level timing for its pins.