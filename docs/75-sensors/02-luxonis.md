---
title: Luxonis camera
pagination_label: Luxonis
description: Luxonis
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Luxonis

## Installation

The Luxonis driver on ROS2 Jazzy is supported by DepthAI v2, it can be installed by running
```bash
sudo apt install ros-jazzy-depthai-ros
```

## Startup

You need to have the following udev rule for the camera to be detected and work properly

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", OWNER="mrs", MODE="0666"
```

Then the launch command will work

```bash
ros2 launch depthai_ros_driver camera.launch.py
```

### In Docker

Place the following compose session snippet into the [example session](70-deployment/10-docker):

```yaml
  # starts the HW API for connecting the MRS UAV System to PX4
  luxonis:
    image: ctumrs/mrs_uav_system:${MRS_UAV_SYSTEM_VERSION}
    network_mode: host
    volumes:
      - /dev:/dev
    privileged: true
    env_file:
      - ./stack.env
    command: bash -c "ros2 launch depthai_ros_driver camera.launch.py namespace:=$UAV_NAME pointcloud.enable:=true"
```

## Useful parameters

- **namespace** prefix for the oak topics e.g namespace:=$UAV_NAME
- **pointcloud.enable** should the node publish a pointcloud topic e.g pointcloud.enable:=true

## More info

Check the following sources

[Luxonis Documentation](https://docs.luxonis.com/software/ros/depthai-ros/driver/)

[GitHub Repository](https://github.com/luxonis/depthai-ros/tree/jazzy)
