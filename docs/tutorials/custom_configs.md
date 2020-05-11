---
layout: default
title: How to use custom config files
parent: Tutorials
---

# How to use custom config files

## Motivation

If you want to change any parameters of the MAV control pipeline such as the speed constraints of a tracker, turn off collision avoidance etc., you shouldn't modify the default values in `mrs_uav_controllers`, `mrs_uav_trackers` etc.
Modifying the values directly in the core packages is a bad idea, because it makes your setup hard to maintain and deploy without conflicts with other people's setups.
The correct way to go about this is to use **custom configuration files** by leveraging the `config_*` argument of the `core.launch` launchfile.
This argument facilitates loading of a `.yaml` file for the various packages in the MAV control pipeline, which overrides the default settings from the respective package configuration files.

## Activating a custom config file

The simplest example of how to use this technique is to create a `.yaml` file with your value of the parameter that you want to change and point the respective argument of `core.launch` to its filepath.
Let's say that you want to change the controller, which will be used after takeoff, to SO(3) (the default is MPC).
Create a file `uav_manager.yaml` in a subfolder `custom_configs` and fill it with the following lines:
```
takeoff:
  after_takeoff:
    controller: "So3Controller"
```
and launch the MAV control pipeline using the command (note that it has to be called from the parent folder of the `custom_configs` subfolder)
```
roslaunch mrs_uav_general core.launch config_uav_manager:=./custom_configs/uav_manager.yaml
```
This will load your custom configuration file and override the default parameters of the `uav_manager` package.

Note that in the standard use-case scenario, you should use either tmuxinator or a tmux script (see, e.g., [the simulation tutorial](simulation/howto) for an explanation) to start your MAV setup.
In that case, you should put the `custom_configs` subfolder in the same path as the tmuxinator/tmux script.
This way, you can prepare a self-contained folder in the repository of your project to start your setup.
Then all is required to transfer and test your setup on an MAV in a real-world experiment is to clone your repository, compile your stuff and start the script.
No need to run around in panic, wondering why collision avoidance is turned on, trying to manually edit the correct file in the respective package to disable it only to have this change overriden the moment someone does a `git pull` in `uav_core` (we've all been there :)!

## Deducing what parameter to change in which file

OK, so you know how to change the after-takeoff controller, but what about the collision avoidance parameters?
What if you want to change the speed constraints of the used tracker?
Well I'm not going to write a how-to for every single parameter in the MAV pipeline - you should learn how to find these and change them yourself.

The process is simple:

1. decide which parameter you want to change,
2. find out in which package to change it,
3. create a config file with your updated value,
4. update the `core.launch` line in your start script with the new file.

**1.** Deciding which parameter to change depends on your specific need and you should probably know it the best, but in case you are unsure, you can consult some senior student from the lab.

**2.** Finding out which package actually takes care of the functionality you want to modify may be a bit more tricky.
You can follow these general guidelines:

* The `mrs_uav_controllers` package contains gains of the MPC, SO(3) etc. controllers.
* The `mrs_uav_odometry` package contains constants for filtering data from the rangefinder for measuring height of the MAV and other self-localization related sensors, and lists the available estimators.
* The `mrs_optic_flow` package contains constants the optical flow self-localization algorithm (you'd know if you were using it).
* The `mrs_uav_trackers` package contains parameters for the various trackers. This is where you'll find the **collision avoidance** parameters (in the `mpc_tracker.yaml` file)!
* The `mrs_uav_managers` package contains various general parameters, such as safety thresholds, lists of available trackers and controllers and so on, but also the **speed/acceleration/jerk/snap constraints** of the trackers.

If you don't find what you're looking for in this short list, you can just go through the packages and look through the files yourself.
You can find the packages in `~/git/uav_core/ros_packages`.

**3.** Create the config file as explained above.
Make sure to keep the same structure of the parameter naming as in the default config file
The namespace nesting in the `yaml` format can be a bit unclear sometimes, but it's important to get it right, otherwise the parameter will not be overriden and the default value will be used!
You should also name the config file the same as was the name of the original file, containing the default value.

**4.** Update your start script as explained above.
Note that you have to change the part of the `config_*` argument of the `core.launch` launchfile after the underscore to the name of the respective config file (without the `.yaml` appendix).
For example if you were to change a parameter, which you've found in the `mpc_tracker.yaml` file, use `config_mpc_tracker:=./custom_configs/mpc_tracker.yaml`.
