---
title: Bluefox2 IMU
pagination_label: Bluefox2 IMU
description: Bluefox2 IMU
---

<!--:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::-->

# Bluefox2 IMU

This is a package for publishing the data from the IMU mounted on a Bluefox2 MRS module.

## Installation

The driver is part of the full installation of the mrs system (`ros-jazzy-mrs-uav-system-full`).
If missing, install it by running the following command (it is part of the `mrs_serial` package together with other nodes):
```bash
sudo apt install ros-jazzy-mrs-serial
```

You also need to install udev rules, so you don't have access permission errors. Copy the udev rule file from package's `install/udev` directory to `/etc/udev/rules.d/` and refresh udev daemon:

```bash
sudo cp install/udev/* /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

## Startup

First, you have to define environment variable 'UAV_NAME'. For example, you can do it by:

```bash
export UAV_NAME=uav1
```

or you can add it to your *.bashrc* to make it more permanent:

```bash
echo "export UAV_NAME=uav1" >> ~/.bashrc
```
If you do not have this environment variable set, you will see this error message:

```bash
Caught exception in launch (see debug for traceback): environment variable 'UAV_NAME' does not exist
```

Now you can launch the IMU node:

```bash
ros2 launch mrs_serial vio_imu.launch.py
```

The default device for the IMU is `/dev/ttyACM0`. If the node can't find this device and you are seeing coresponding error messages in the terminal, you can search for the IMU device and specify it:

```bash
ros2 launch mrs_serial vio_imu.launch.py device:=/dev/tty...
```

Alternatively, you can define the udev rule for your device.

## Custom configs

You can provide a custom config to the launch file as
```
ros2 launch bluefox2 single.launch.py custom_config:=custom_config.yaml
```

The custom configs allows you to define your own parameters and also to remap the outgoing topics:
```yaml
uav_name: uav1
enable_profiler: false
portname: /dev/ttyACM0
baudrate: 460800
verbose: false
use_timeout: false
serial_rate: 460800
desired_publish_rate: 200   # 1000, 500, 200, 100, 50... floor(1000/desired_publish_rate)
```

Note: You cannot specify namespace in your custom config because this node is running as a composable node.

The `desired_publish_rate` determines how many messages will be skipped. If it is set to `500`, then every other message is published. If it is set to 100, then only every 10th message is published. Skip factor is computed as `floor(1000/desired_publish_rate)`, so it makes sense to only use the integer divisors: 1000, 500, 200, 100, 50...

### In Docker

Place the following compose session snippet into the [example session](70-deployment/10-docker):

```yaml
  bluefox_imu:
    image: ctumrs/mrs_uav_system:stable
    network_mode: host
    volumes:
      - /dev/:/dev/ # besides setting 'privileged' flag to true, you also have to mount '/dev' to see device file
    privileged: true
    env_file:
      - ./stack.env
    command: bash -c "ros2 launch mrs_serial vio_imu.launch.py"
```

## More info

Check the following sources

[GitHub Repository](https://github.com/ctu-mrs/mrs_serial/tree/ros2)
