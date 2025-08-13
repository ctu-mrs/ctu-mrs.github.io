---
title: Realsense
pagination_label: Realsense camera
description: Realsense camera
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Realsense

MRS has our own realsense wrapper package with custom launch files that provided the `custom config` mechanism.

## Installation

The driver is part of the full installation of the mrs system (`ros-jazzy-mrs-uav-system-full`).
If missing, install it by running the following command:
```bash
sudo apt install ros-jazzy-mrs-realsense
```

## Startup

```bash
ros2 launch mrs_realsense uav.launch.py
```

## Custom configs

You can provide a custom config to the launch file as
```
... custom_config:=custom_config.yaml
```

The custom configs allow you to remap the outgoing topics:
```yaml
/uav1/rgbd:
  ros__parameters:
    remappings:
      '~/depth/image_rect_raw': ~/depth/image_raw
      '~/depth/image_rect_raw/compressed': ~/depth/image_raw/compressed
      '~/depth/image_rect_raw/compressedDepth': ~/depth/image_raw/compressedDepth
```

## More info

Check the following sources

[GitHub Repository](https://github.com/ctu-mrs/realsense/tree/ros2)
