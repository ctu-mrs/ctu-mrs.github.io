---
title: Installation
pagination_label: Installing the FlightForge simulator
description: How to install the FlightForge simulator
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Installation

The FlightForge simulator is composed of two main parts - the simulator itself either as a binary or as a Unreal engine plugin and the ROS package that interfaces with the simulator and provides the necessary ROS topics and services.

## Prerequisites

1. The [ROS Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) installed.
2. The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) installed.

## FlightForge Simulator 

The prebuild binaries of the simulator can be downloaded from the [here](https://nasmrs.felk.cvut.cz/index.php/s/MnGARsSwnpeVy5z). 
The simulator is available for Linux and Windows, the simulator can be run by running the binary file `mrs_flight_forge.sh` in the downloaded archive.
The binary provides the simulator as a standalone application that can be run without the need of Unreal Engine.
However, if you wish to create a custom environment or modify the simulator you can download our Unreal Engine plugin from [flight_forge repository](https://github.com/ctu-mrs/flight_forge), and place it in the `Plugins` folder of your Unreal Engine project.
Currently, the plugin is only guaranteed to work with Unreal Engine 5.4.

## ROS Package

To interface with the FlightForge simulator, you need to install the ROS package that provides the dynamics and publishes the necessary ROS topics and services.
The ROS package is either installed when installing the MRS UAV System Full or can be installed separately by running the following command:

```bash
sudo apt install ros-jazzy-mrs-uav-flightforge-simulation
```

If you wish to modify the ROS package, you can clone the repository from [mrs_uav_flightforge_simulator](https://github.com/ctu-mrs/mrs_uav_flightforge_simulator/tree/ros2) and place it in your workspace.
