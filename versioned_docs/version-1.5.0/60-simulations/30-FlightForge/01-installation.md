---
title: Installation
pagination_label: Installing the FlightForge simulator
description: How to install the FlightForge simulator
---

# Installation

The FlightForge simulator is composed of two main parts - the simulator itself either as a binary or as a Unreal engine plugin and the ROS package that interfaces with the simulator and provides the necessary ROS topics and services.

## Prerequisites

1. The [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed.
2. The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) installed.

## FlightForge Simulator 

The prebuild binaries of the simulator can be downloaded from the [here](https://nasmrs.fel.cvut.cz/index.php/s/MnGARsSwnpeVy5z). 
The simulator is available for Linux and Windows, the simulator can be run by running the binary file `FlightForge.sh` in the downloaded archive.
The binary provides the simulator as a standalone application that can be run without the need of Unreal Engine.
However, if you wish to create a custom environment or modify the simulator you can download our Unreal Engine plugin from [flight_forge repository](https://github.com/ctu-mrs/flight_forge), and place it in the `Plugins` folder of your Unreal Engine project.
Currently, the plugin is only guaranteed to work with Unreal Engine 5.4.

## ROS Package

To interface with the FlightForge simulator, you need to install the ROS package that provides the dynamics and publishes the necessary ROS topics and services.
The ROS package is either installed when installing the MRS UAV System Full or can be installed separately by running the following command:

```bash
sudo apt install ros-noetic-mrs-uav-unreal-simulation
```

If you wish to develop the ROS package, you can clone the repository from [mrs_uav_unreal_simulation](https://github.com/ctu-mrs/mrs_uav_unreal_simulation) and place it in your workspace.
