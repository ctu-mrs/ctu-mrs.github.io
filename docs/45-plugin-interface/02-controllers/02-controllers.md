---
title: Controller plugins
pagination_label: API for the controller plugins
description: API for the controller plugins
---

# The Controller plugin interface

The Controller plugin takes current UAV state and command from the [Reference tracker](https://ctu-mrs.github.io/docs/features/trackers/) and provides the desired control ouptuts, which are sent to the flight controller (Embedded autopilot block in diagram below) via [Hardware API](https://ctu-mrs.github.io/docs/plugin-interface/hardware-api/).
Diagram below shows the data architecture of the MRS system where the block containing the Controller plugin is marked in red.

The controller plugin is compiled as *ROS plugins* ([http://wiki.ros.org/pluginlib](http://wiki.ros.org/pluginlib)) with the [interface](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/include/mrs_uav_managers/controller.h) defined by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers).
A controller plugin from any ROS package can be loaded dynamically by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) without it being present during [control manager](https://github.com/ctu-mrs/mrs_uav_managers)'s compile time.
Loaded controllers can be switched by the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) in mid-flight, which allows safe testing of new controllers and adds flexibility to the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system).

![](./fig/diagram_of_system_architecture.jpg)

## Controller plugin - outputs 
The controller plugin can output any of the following control outputs:

  * individual Actuators' throttle
  * Control groups
  * body-frame Attitude rate + Throttle
  * 3D world-frame Attitude + Throttle
  * 3D body-frame Acceleration + Heading rate
  * 3D body-frame Acceleration + Heading
  * 3D body-frame Velocity + Heading rate
  * 3D body-frame Velocity + Heading
  * 3D Position + Heading
  
In the controller plugin source code, you can decide what to return, but it must be available in [Hardware API](https://ctu-mrs.github.io/docs/plugin-interface/hardware-api/). 

## Example controller plugin 

An example of a cutom controller plugin can be found at [this link](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/controller_plugin).
It is highly reccomended to base your controller plugin on this example.
In the example, there is also a [tmux folder](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/controller_plugin/tmux) containing a script that starts the simulation of a UAV in the [Gazebo simulator](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation), where the UAV uses the [example controller plugin](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/controller_plugin).
You have to install [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system) first to run the example.
To load the controller plugin into the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system), you need to provide a custom config file to the [MRS UAV Core](https://github.com/ctu-mrs/mrs_uav_core) containing the parameters for the controller plugin.
The custom config file should contain the following lines (taken from [custom config file of example controller plugin](https://github.com/ctu-mrs/mrs_core_examples/blob/master/cpp/controller_plugin/tmux/config/custom_config.yaml)):
```yaml 
mrs_uav_managers:

  control_manager:

    ExampleController:
      address: "example_controller_plugin/ExampleController"
      namespace: "example_controller"
      eland_threshold: 20.0 # [m], position error triggering eland
      failsafe_threshold: 30.0 # [m], position error triggering failsafe land
      odometry_innovation_threshold: 1.5 # [m], position odometry innovation threshold
      human_switchable: true

      # which outputs the controller can provide
      outputs:
        actuators:             false
        control_group:         false
        attitude_rate:         false
        attitude:              true
        acceleration_hdg_rate: false
        acceleration_hdg:      false
        velocity_hdg_rate:     false
        velocity_hdg:          false
        position:              false

    # list of names of dynamically loaded controllers
    controllers : [
      "ExampleController",
    ]
```

