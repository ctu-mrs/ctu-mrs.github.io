---
layout: default
title: Trajectories
parent: The UAV system
nav_order: 4
---

## Loading trajectories

The `mrs-ctu/trajectory_loader` package contains all information you need for loading a trajectory from a file: [documentation](https://github.com/ctu-mrs/trajectory_loader), [example trajectories](https://github.com/ctu-mrs/trajectory_loader/tree/master/sample_trajectories), and [launch files](https://github.com/ctu-mrs/trajectory_loader/tree/master/launch).
The trajectory loaded with `mrs-ctu/trajectory_loader` will be input to the current [reference tracker](https://ctu-mrs.github.io/docs/software/uav_core/mrs_uav_trackers/), which will generate all-state and feasible reference for the [controller](https://ctu-mrs.github.io/docs/software/uav_core/mrs_uav_controllers/).

## Generating trajectories

You can generate your own trajectories and publish them to `control_manager/trajectory_reference` to fly them (see [UAV-ROS interface](https://ctu-mrs.github.io/docs/system/uav_ros_interface.html#selected-services-for-program-to-machine-interaction)).
This pipeline follows the same scheme `trajectory->tracker->controller` as if the trajectory was loaded with `mrs-ctu/trajectory_loader`.
If a trajectory is not feasible under dynamic constraints of a UAV, the resulting trajectory may be degraded.

To generate feasible time-parametrized trajectories from a desired waypoint path, you can use `mrs_uav_trajectory_generation`.
See [mrs_uav_trajectory_generation documentation](https://github.com/ctu-mrs/mrs_uav_trajectory_generation) for more information about generating trajectories under dynamic constraints and [list of topics/services](https://ctu-mrs.github.io/docs/system/uav_ros_interface.html#trajectory-generation) for using the `mrs_uav_trajectory_generation` package.
