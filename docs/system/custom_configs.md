---
layout: default
title: Custom config files
parent: The UAV system
nav_order: 4
---

# How to use custom config files for uav_core modules

## Motivation

There are many configuration files in the [uav_core](https://github.com/ctu-mrs/uav_core).
They contain many options to modify the behavior of the control and estimation pipeline.
Since everyone needs to change some values for his experiments, the uav_core modules contain only a *general defaults* for each supported UAV type.
In this tutorial, we provide an efficient way ho customize the configuration **without the need to change** any files in the uav_core repository.
It is done by creating a custom configuration files which contain only the option you would like to change from the default values.
Those **custom configs** can be loaded over the defaults without the need to change the uav_core itself.

## The config file hierarchy

The ROS launch files in the **uav_core** load three levels of configuration files.
1. A topmost general configuration, typically stored in a **default** subfolder of the packages.
2. A UAV-type specific configuration, which is already divided into two groups: for **simulation** and the real **uav**. The particular files are automatically selected using the environment variables `$UAV_TYPE` and `$RUN_TYPE`.
3. A user-defined **custom config** file provided through the launch file argument, which is the subject of this manual.

## Activating a custom config file

We assume you already have a custom *tmuxinator* session, as suggested by this [manual](https://ctu-mrs.github.io/docs/simulation/howto.html).
Let's say that you want to change the controller, which will be used after takeoff.
This option is originally defined in [uav_manager.yaml](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/control_manager.yaml), in the [mrs_uav_manager](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/uav_manager.yaml) package.
An example of how to use this technique is to create a `.yaml` file with the value of the parameter that you want to change.
The file is then passed to the [core.launch](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/core.launch) launchfile.
Create a file `uav_manager.yaml` in a subfolder `custom_configs` within your tmuxinator session folder and fill it with the following lines:
```
takeoff:
  after_takeoff:
    controller: "Se3Controller"
```
Then add an optional argument to the [core.launch](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/core.launch) line in your tmuxinator session:
```
roslaunch mrs_uav_general core.launch config_uav_manager:=./custom_configs/uav_manager.yaml
```
This will load your custom configuration file and it will override the default parameters from the [mrs_uav_managers](https://github.com/ctu-mrs/mrs_uav_managers) package.

We strongly suggest using [tmuxinator](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts) (for simulation) or a [tmux](https://github.com/ctu-mrs/uav_core/tree/master/tmux_scripts) (for a real UAV) scripts for starting the simulation.
In both cases, we usually create a subfolder `custom_configs` in the same path as the tmuxinator/tmux script.
This way, you can prepare a self-contained folder in your own repository, which you can save, version it with git and change independently on the uav_core.

**The lesson** from this should be:

* never edit config files directly in somewhere in [uav_core],
* always keep the **custom config** files at your tmux/tmuxinator session.

## Which parameters should I change?

Deducing which parameters you want to change depends on your particular scenario.
We recommend to go through the description of all our packages ([link](https://ctu-mrs.github.io/docs/software/uav_core)) and familiarize yourself with their functions.
Their config files are typically contained within the `config/default` subfolder of each repository.

## How should I name the *core.launch* argument?

Each program in each package the `[core.launch](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/core.launch)` has an argument dedicated to the program's custom config file.
The simplest way how to find the argument is to look into the launch file.
However, generally, the argument name has a pattern `config_node_name`.
E.g., `config_control_manager`, `config_odometry` or `config_mpc_controller`.

## Complete examples

An example dedicated to **custom configs** is located within our tmux session examples:
[https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts)
under the name `one_drone_tmuxinator_custom_configs`.

Also, the [mrs_uav_testing](https://github.com/ctu-mrs/mrs_uav_testing) package has its test sessions customized this way.
See [https://github.com/ctu-mrs/mrs_uav_testing/tree/master/tmux](https://github.com/ctu-mrs/mrs_uav_testing/tree/master/tmux).
