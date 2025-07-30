---
title: MRS System Launch pipeline
pagination_label: MRS System Launch Pipeline
descriptin: What happens when you launch the system
---

# MRS System Launch Pipeline

- When you first launch the system, the `core` launches various managers of the system. All of these managers are nodes that contain plugins that are loaded at runtime and they all share a parameter loader. These nodes are:
  - `control manager`: Loads the following plugins(controllers and tracker) and loads parameters that serve all the plugins.
    - `Se3Controller`
    - `MpcController`
    - `MpcTracker`
    - `LineTracker`
  - `estimation manager`: Loads various Kalman filters and estimators depending on various sensors:
    - `Gps Baro`
    - `Passthrough`
    - `RTK`
  - `constraint manager`: Handles the constraints for the system and passes them to estimators, controllers, and trackers
  - `gain manager`
  - `transform manager`
  - `uav manager`: Manages the UAV specific configurations and ties the system together


## Param Loader

The param loader is a special component of `mrs_lib` and serves as an evolution of the usual ROS parameter server. The parameter server is not shared for the entire system and instead, is initialised and parameters are populated per node. Unlike the ROS parameter server, param loader does not maintain a global parameter dictionary but instead returns the parameters in the same order in which they are loaded from a file. So, if a requested parameter is not found in the first loaded file, it moves to the second loaded file and so on. When none of these files return the requested parameter, the loader searches the ROS parameter server for the parameter.

As such, when you launch the MRS system, the parameters are loaded in every node for itself and passed to each plugin inside a node. The order of loading and returning the parameters is as follows:

- `custom_config`: The custom configuration of the system specified in the launch file. Given hightest priority.
- `platform_config`: The platform config is specific to each UAV of the system and is loaded and retrieved with second highest priority.
- `world_config`: The config containing the origin, co-ordinates, and safety area specifications.
- `network_config`
- Internal yaml config files specified in the plugins.



