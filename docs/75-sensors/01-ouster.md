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

## Finding out IP address of the sensor

You have to first find out what is the IP address of your Ouster lidar, so you can use it later as the parameter to the driver's launchfile. Ouster lidars used by MRS have their IP addresses set statically in the network `10.10.20.0/24`. First, to see the sensor on your network, you have to set the static IP address of your ethernet interface. Here is the example:

![KDE desktop environment network config utility](fig/ouster-static-ip-setting.png)

Now you can run the `nmap` command, to search the network for sensor device:

```bash
$ nmap -sn 10.10.20.0/24
```

You should get the output similar to this:

```bash
Starting Nmap 7.94SVN ( https://nmap.org ) at 2025-10-24 14:59 CEST
Nmap scan report for hermiodth (10.10.20.1)
Host is up (0.00028s latency).
Nmap scan report for 10.10.20.90
Host is up (0.0010s latency).
Nmap done: 256 IP addresses (2 hosts up) scanned in 2.61 seconds
```

Here, we can see our sensor has IP address `10.10.20.90`. You can now use it to launch the driver or to open the sensor's web interface (http://10.10.20.90/), where you can configure the sensor.

## Startup

```bash
ros2 launch ouster_ros sensor.composite.launch.py sensor_hostname:=10.10.20.90 udp_dest:=10.10.20.1
```

- the **udp_dest** parameter is the IP address (hostname) of your or drone's computer
- the **sensor_hostname** is the IP address (hostname) of the sensor

### In Docker

Place the following compose session snippet into the [example session](../70-deployment/10-docker/index.md):

```yaml
  ouster:
    image: ctumrs/mrs_uav_system:${MRS_UAV_SYSTEM_VERSION}
    network_mode: host
    volumes:
      - /dev:/dev
    privileged: true
    env_file:
      - ./stack.env
    command: bash -c "ros2 launch ouster_ros sensor.composite.launch.py sensor_hostname:=10.10.20.90 udp_dest:=10.10.20.101 viz:=false"
```

## Useful parameters

- **viz** decides if the ros visualizer will be started alongside the node e.g viz:=false
