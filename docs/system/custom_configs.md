---
layout: default
title: Configuring the MRS UAV System
parent: The UAV System
nav_order: 2
---

| :warning: **Attention please: This page needs work.**                                                                                             |
| :---                                                                                                                                              |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# Motivation

There are many configuration files _baked_ in the [mrs_uav_core](https://github.com/ctu-mrs/mrs_uav_core) and other packages of the system.
They contain many options for modifying the behavior of the system and users are **encouraged** to configure the system to their needs.
In this tutorial, we provide an efficient way ho customize the configuration **without the need to change** any files within the installed system.

## The concept of a _custom config_ file

Each launch file within our system loads appropriate config files from its package.
Changing these config files directly is **not recommended** as well as **copying them elsewehere and changing the launch file**.
Instead, a parameter can be used to pass a **custom config** file that only overrides the values that the used wants to change relative to the defaults.
This mechanism allows the user to store the changes locally.
In general, our launchfiles accept the argument
```
custom_config:=<path to the file>
```
which is used to pass the path to the custom config.

## core.launch configs

The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) is launched throught the `core.launch` launch file.
The `core.launch` requires the following 2 custom configuration files:

## The Platform Config

This file is supposed to override the defaults for a particular UAV platform.
Usually, this file is provided by us with each simulated (and realword) UAV.
The platform configs are stored in a location logically close to the source of the simulated UAV, e.g.,

* In [mrs_uav_gazebo_simulation](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/tree/master/ros_packages/mrs_uav_gazebo_simulation/config/mrs_uav_system) for our Gazebo simulation,
* In [mrs_multirotor_simulator](https://github.com/ctu-mrs/mrs_multirotor_simulator/tree/master/config/mrs_uav_system) for our MRS simulation,
* In [mrs_uav_coppelia_simulation](https://github.com/ctu-mrs/mrs_uav_coppelia_simulation/tree/master/mrs_uav_coppelia_simulation/config/mrs_uav_system) for our Coppelia simulation,
* In [mrs_uav_dji_tello_api](https://github.com/ctu-mrs/mrs_uav_dji_tello_api/tree/master/config/mrs_uav_system) for our DJI Tello interface.
* In [mrs_uav_deployment](https://github.com/ctu-mrs/mrs_uav_deployment/tree/master/config/mrs_uav_system) for our realword drones.

## The Custom Config

This file is dedicated for the user's relative changes to the default configuration that was already modified by the _platform config_.

## Which parameters can be changed?

All parameters that can be changed by the users can be listed by
```bash
rosrun mrs_uav_core get_public_params.py
```
The parameters are _scraped_ from the currently installed packages from the system, so the list is always going to be compatible with your current installation.

For convenience, the parameters can be piped into a file and later inspected:
```bash
rosrun mrs_uav_core get_public_params.py > /tmp/params.yaml
```
or piped to less:
```bash
rosrun mrs_uav_core get_public_params.py | less
```
or piped to vim:
```bash
vim <(rosrun mrs_uav_core get_public_params.py)
```

## Example

All the example tmux sessions that we provide are already pre-set with the _platform configs_ and the _custom config_ file.
