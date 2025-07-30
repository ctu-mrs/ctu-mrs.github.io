---
title: Post installation guide
---

# Linux SWAP size increase

When compiling the workspace on a machine with multiple cores, the memory gets full quite fast, which leads to freezing of the whole OS.
One solution is to build with less threads using the `-j` compile flag (e.g. `catkin build -j4`)
Other solution is to increase the size of the swap file, which extends the available memory for the build process.

Perform these steps to increase the size of the swap file.

```bash
# Turn swap off
# This moves stuff in swap to the main memory and might take several minutes
sudo swapoff -a

# Create an empty swapfile
# Note that "1M" is basically just the unit and count is an integer.
# Together, they define the size. In this case 16GiB.
sudo dd if=/dev/zero of=/swapfile bs=1G count=16

# Set the correct permissions
sudo chmod 600 /swapfile

# Set up a Linux swap area
sudo mkswap /swapfile 

# Turn the swap on
sudo swapon /swapfile

# Check if the swap is now the desired size
grep SwapTotal /proc/meminfo
```

To make the changes permanent (persist on restarts), add this line to the end of your `/etc/fstab`:

```bash
/swapfile none swap sw 0 0
```

# Disabling automatic sleep/hibernation

TODO

```bash
~/git/uav_core/miscellaneous/scripts/disable_hibernation.sh
```
OR
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

# Change network interface names to the `nice ones`

TODO

Check [UAV core network settings](https://github.com/ctu-mrs/uav_core/tree/master/miscellaneous/network_settings)

# Disabling network manager

TODO

```bash
~/git/uav_core/miscellaneous/scripts/disable_network_manager.sh
```
OR
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

# udev rules

Similar to [PX4 ROS setup](https://ctu-mrs.github.io/docs/hardware/px4_configuration.html#ros-setup)

```bash
cd /etc/udev/rules.d
```
Create a new file (with sudo privileges) and call it `99-usb-serial.rules`. Paste this line into the file:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="YOUR_DEVICE_DESIGNATOR",OWNER="mrs",MODE="0666"
```

Replace idVendor, idProduct with you values, and change the OWNER name to your user name. You can use the following utility:

```bash
lsusb
```

Now, if you disconnect and reconnect the device, it should show up as `/dev/YOUR_DEVICE_DESIGNATOR`. Now you can try

```bash
cat /dev/YOUR_DEVICE_DESIGNATOR
```

and you should see some incoming messages.

Our HW guys have already prepared `.rules` files for our most used distribution boards and they can be found in

```bash
~/git/uav_core/miscellaneous/udev_rules/
```
so feel free to copy them to your `/etc/udev/rules.d` folder and (re)use them.

Here are some devices you need to set rules for.

## FTDI

## RPLidar

## Arduino

# Time synchronization

TODO

# NUC Power limits in BIOS

TODO

# Setup and validate .bashrc

TODO
