---
title: Ouster
pagination_label: Ouster
description: Ouster
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Ouster

## Installation

The Ouster driver is part of the full installation of the MRS UAV System.
If you are still missing it, run
```bash
sudo apt install ros-jazzy-ouster-ros
```

## Startup

```bash
ros2 launch ouster_ros sensor.composite.launch.py sensor_hostname:=10.10.20.90 udp_dest:=192.168.69.101
```

- the **udp_dest** parameter is the IP address (hostname) of the drone's computer
- the **sensor_hostname** is the IP address (hostname) of the sensor

## Useful parameters

- **viz** decides if the ros visualizer will be started alongside the node e.g viz:=false
