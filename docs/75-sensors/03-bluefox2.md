---
title: Bluefox2
pagination_label: Bluefox2 camera
description: Bluefox2 camera
---

<!-->:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::-->

# Bluefox2

This a refactored version of the old Bluefox2 camera driver for ROS2 Jazzy. Original ROS version can be found here: https://github.com/ctu-mrs/bluefox2

## Installation

The driver is part of the full installation of the mrs system (`ros-jazzy-mrs-uav-system-full`).
If missing, install it by running the following command:
```bash
sudo apt install ros-jazzy-bluefox2
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

Now you can launch the camera node. This launch command should find the camera automatically, so you don't have to specify camera serial number manually.

```bash
ros2 launch bluefox2 single.launch.py
```

If camera discovery went fine, you should see something like this:

```bash
Found Bluefox2 devices: [25003671 12345678... ]
```

The first found device is used. If there are some problems, you can also list cameras manually with more info printed:

```bash
ros2 run bluefox2 bluefox2_list_cameras_human_readable
```

If you have more cameras connected and you don't want the driver to use the first camera (or when discovery fails for some reason), you can to specify camera serial number manually:

```bash
ros2 launch bluefox2 single.launch.py device:=12345678
```

or you can set the environment variable 'BLUEFOX' to the value of serial number:

```bash
export BLUEFOX=12345678
```

The driver will pick it up automatically.

## Custom configs

You can provide a custom config to the launch file as
```
ros2 launch bluefox2 single.launch.py custom_config:=custom_config.yaml
```

The custom configs allows you to define your own parameters and also to remap the outgoing topics:
```yaml
fps: 30 # limit the camera framerate
remappings: # remap topics in case your software searches for specific topic
  '~/depth/image_rect_raw': ~/depth/image_raw
  '~/depth/image_rect_raw/compressed': ~/depth/image_raw/compressed
  '~/depth/image_rect_raw/compressedDepth': ~/depth/image_raw/compressedDepth
```

Note: You cannot specify namespace in your custom config because this node is running as a composable node.

## More info

Check the following sources

[GitHub Repository](https://github.com/ctu-mrs/bluefox2/tree/ros2)
