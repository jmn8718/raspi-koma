# UBUNTU SERVER 24.04.3

By default, raspberry camera v2 is not supported on ubuntu 24.04, so it is required to build the dependecies on the system.

1. Install Missing IPA Modules
The most common fix is ensuring the Raspberry Pi-specific IPA packages are installed. Ubuntu separates these from the core libcamera library. Run the following:
```sh
sudo apt install libcamera-ipa-raspberrypi libcamera0.2
```

2. Verify the Kernel/Firmware Configuration
Since you are on a Pi 3B+, the system needs to explicitly load the overlay for the IMX219 sensor. Check your /boot/firmware/config.txt (or /boot/config.txt) file:

Open the file: `sudo nano /boot/firmware/config.txt`

Ensure these lines are present and not commented out:

```
camera_auto_detect=1
dtoverlay=imx219
```

3. GPU Memory Allocation

The Pi 3B+ has limited RAM. Ensure you have allocated enough memory to the GPU for camera processing. In your config.txt, ensure you have:

```
gpu_mem=128
```

4. After making these changes, you must reboot:

```
sudo reboot
```

Once rebooted, try listing the cameras again with `cam -l`.

### Build the RPi Fork

1. Install Dependencies

First, install the tools needed to compile the software:

```sh
sudo apt install -y git cmake meson ninja-build build-essential \
python3-pip python3-yaml python3-ply libboost-dev libgnutls28-dev \
openssl libjpeg-dev libtiff5-dev libpng-dev libdrm-dev libexpat1-dev \
libcamera-dev v4l-utils pybind11-dev libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libboost-program-options-dev
```

2. Build the libcamera Raspberry Pi Fork

```sh
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

# Setup again
meson setup build --buildtype=release -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled

# Build using ONLY ONE core (-j 1)
ninja -C build -j 1

# Install
sudo ninja -C build install
```

3. Build the Camera Apps (rpicam-apps)

```sh
cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build --buildtype=release
ninja -C build -j 1
sudo ninja -C build install
sudo ldconfig
```

*Testing the Fix*

After rebooting, try the new command:
`rpicam-hello --list-cameras`

If it lists your IMX219, you can take a test photo with:
`rpicam-still -o test.jpg`

*Final Verification*

After the installation completes, verify that the library is correctly linked:
`ldconfig -p | grep libcamera`