---
title: Raspberry Pi 5
pagination_label: Raspberry Pi 5
description: Raspberry Pi 5
---

This repository contains our knowledge base on Raspbery Pi 5.

# Camera firmware issues

For cameras based on `ov9281` connected via the MIPI port, such as the Arducam UC-599 Rev-B, you need to set up explicit firmware overlay. This is set up in `/boot/firmware/config.txt`. You need to disable automatic detection of cameras by commenting out or setting to `0` the line:
```
camera_auto_detect=1
```
and adding the following lines into the `[all]` section:
```
dtoverlay=ov9281
dtoverlay=ov9281,cam0
dtoverlay=ov9281,cam1
```
The `cam#` lines set the camera to be detectable on both the MIPI ports.

It may also be necessary to comment out the line
```
dtoverlay=arducam-pivariety
```

## Warning

Do not install the `vcdbg` package - it will remove the overlays needed for MIPI camera use.
It seems that the package is currently only compatible with older versions of the Raspberry Pi.
If you did this, remove all packages that were installed with `vcdbg` and reinstall `raspi-firmware`.

# Using GPU acceleration in Singularity/Apptainer

By default, the Ubuntu 20.04 Singularity image we use to run ROS on the device has older MESA drivers that do not support hardware acceleration on the onboard Broadcom V3D 7.1 GPU.
This can be overcome by manually adding a non-official ppa to upgrade the drivers in Singularity.

First, you need an overlay with enough space - at least 2GB should do it. If you already have an overlay image file that is too small, you can expand it using:
```
e2fsck -f overlay.img
resize2fs overlay.img 2000M
```
Next, load up the Singularity container with root privileges using
```
sudo ./wrapper.sh
```
and when inside do the following:
```
sudo apt-get install software-properties-common # adds the missing add-apt-repository command
chmod 1777 /tmp # necessary, since adding the ppa will be making temporary files here and by default it is not writable in the container
add-apt-repository ppa:kisak/kisak-mesa # press Enter when prompted
sudo apt install libglx-mesa0 libgl1-mesa-dri
```
This should have upgraded the MESA drivers to the newest version.
Lastly, note that the V3D 7.1 GPU only supports GLSL 3.10 - if you are using a shader that explicitly requires a newer version, try manually rewriting the requirement in the code, it may work. If not, you may be out of luck.

# Using WiringPi in Singularity/Apptainer

In order to control the GPIO pins inside of 20.04 Ubuntu Singularity image, you need to install an up-to-date version of the library inside the container.
These steps worked for me:

1. Enter the wrapper with `sudo`
2. Download new versions of `autoconf` and `autoconf-archive`:
```
wget ftp.de.debian.org/debian/pool/main/a/autoconf/autoconf_2.71-3_all.deb
wget ftp.de.debian.org/debian/pool/main/a/autoconf-archive/autoconf-archive_20220903-3_all.deb
dpkg -i ./autoconf*deb
```
4. Build and install `libgpiod`:
```
git clone https://github.com/brgl/libgpiod.git
cd libgpiod
./autogen.sh --enable-tools=yes --prefix=/usr
make
make install
```
5. Copy (and replace if there is one already) `libgpoiod/lib/uapi/gpio.h` from the git directory to `/usr/include/linux/`
6. Build and install new `WiringPi`:
```
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build debian
sudo dpkg -i ./debian-template/wiringpi-*.deb
```
