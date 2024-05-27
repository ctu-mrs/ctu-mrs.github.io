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

# Using GPU acceleration in Singularity
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
chmod 1777 /tmp # necessary, since adding the ppa will be making temprary files here and by default it is not writable in the container
add-apt-repository ppa:kisak/kisak-mesa # press Enter when prompted
sudo apt install libglx-mesa0 libgl1-mesa-dri
```
This should have upgraded the MESA drivers to the newest version.
Lastly, note that the V3D 7.1 GPU only supports GLSL 3.10 - if you are using a shader that explicitly requires a newer version, try manually rewriting the requirement in the code, it may work. If not, you may be out of luck.
