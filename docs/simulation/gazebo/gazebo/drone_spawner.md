---
layout: default
title: MRS Drone Spawner
parent: Gazebo
grand_parent: Simulation
nav_order: 2
---

| :warning: **Attention please: This page is outdated.**                                                                                           |
| :---                                                                                                                                             |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# About

The `mrs_drone_spawner` is a ROS node written in Python, which allows you to dynamically add drones into your Gazebo simulation.
At the core, the spawner handles three basic operations:

* running the [PX4 Firmware SITL](https://dev.px4.io/master/en/simulation/ros_interface.html) (software-in-the-loop)
* adding a [vehicle model](https://github.com/ctu-mrs/mrs_uav_system#unmanned-aerial-vehicles) into the Gazebo world
* running [MAVROS](http://wiki.ros.org/mavros), which handles the communication (MAVLink) between the firmware and the simulator

These three processes have to be started sequentially in this specific order. Disrupting the order may cause the simulator to freeze or crash.

# Spawning a drone

The `mrs_drone_spawner` is **started automatically** with the simulator when starting Gazebo with our launch file
```bash
roslaunch mrs_simulation simulation.launch
```

If you use a custom launch file to start Gazebo, you may need to start the spawner node **manually** by calling
```bash
roslaunch mrs_simulation mrs_drone_spawner.launch
```

To add a vehicle into the simulation, call a service provided by the drone spawner
```bash
rosservice call /mrs_drone_spawner/spawn "1 --enable-rangefinder"
```

The service takes a **string** as an argument. Use the string to pass arguments to the spawner and define the vehicle configuration.
The list of all available parameters can be displayed by dry-running the spawner script without arguments
```bash
rosrun mrs_simulation mrs_drone_spawner
```
This will only list the available parameters and their description and exit the program. You can safely perform the dry-run even if an instance of the `mrs_drone_spawner` is already running.
When passing multiple parameters to the node, the following structure is used:

* `1 2 3 ...` - integers ranging from 0 to 250 define ID of the vehicle. The model will be named as `uavID` in Gazebo. Use multiple numbers to spawn more than one vehicle at once
* `--f450` - change the type of vehicle to be spawned (default is t650, available options are `--f450`, `--f550` and `--t650`)
* `--param-name` - everything started with `--` is treated as a new parameter. If you follow it up with variables without the `--`, it will be assigned into child variables of this parameter. Parameters without children are treated as *booleans*, and set the value of the param-name to `True`. It is a good practise to name the *bool* parameters as `--enable-something` or `--use-something`.
* `--pos x y z heading` - specify the spawn position of the vehicle. If multiple vehicles are spawned with one command, this position will be assigned to the first vehicle and the following vehicles will be placed in an array next to it (along x-axis, 2 meter spacing)
* `--pos_file path_to_file` - specify the spawn position in a `.yaml` or `.csv` file. The position file may define specific positions for many drones. **Absolute filepath** is required. `pwd` can be used when calling spawner in directory containing the file such as ``--pos_file `pwd`/pos_file.csv``.

**Note:** It is permitted to use **both** `-` and `_` as word separators in the input parameters (`--enable_rangefinder` and `--enable-rangefinder` are both valid).

**Example:** spawn two vehicles with IDs 1 and 7, change vehicle type to f450, add a laser rangefinder and an Ouster 3D LiDAR, specify the LiDAR type to OS1-32 and use GPU acceleration of the LiDAR ray-tracing. All vehicles spawned via this the command will have the same configuration.

```bash
rosservice call /mrs_drone_spawner/spawn "1 7 --f450 --enable-rangefinder --enable-ouster --ouster-model OS1-32 --use-gpu-ray
```

## Tips
* Using a blank space instead of a number will assign the vehicle an ID from unused numbers.
* The spawner will **not** attempt to create a new vehicle, if the desired **ID is already in use**.
* The service calls in Python are handled asynchronously. Please **wait** until the previous spawn command is dealt with **before issuing a new one**. Disrupting the spawn procedure may lead to Gazebo freezing or crashing.
* If you want ground truth odometry to be published or use the `RTK` estimator, add the parameter `--enable-ground-truth`.

# Advanced user zone

## Command line arguments

When starting the spawner manually, you can use the following command-line arguments:
* `no_help` - will *not* display the list of available parameters and instead *start the ROS node*
* `verbose` - will display the list of available parameters after the node is started
* The [`mrs_drone_spawner.launch`](https://github.com/ctu-mrs/mrs_simulation/blob/master/launch/mrs_drone_spawner.launch) uses both `no_help` and `verbose` by default.

## Parameter definitions, defining your own sensors

The available spawner parameters are defined in the [`spawner_params.yaml`](https://github.com/ctu-mrs/mrs_simulation/blob/master/config/spawner_params.yaml) file.
Note that not all sensors are available for all the vehicle types.
The config file stores the available configurations in the following format: `parameter: [default_value, help_description, [compatible_vehicles]]`
The list of compatible vehicles and parameter names have to match the model definitions, which are contained in the [`.xacro`](https://github.com/ctu-mrs/mrs_simulation/tree/master/models/mrs_robots_description/urdf) files.
All parameters passed into the `mrs_drone_spawner` may be accessed in the `.xacro` in a pythonic dictionary `optionals`.
To access the value of a specific parameter inside the `.xacro`, use `${optionals['param_name']}`.

When **adding** a brand new sensor or vehicle configuration, it is **your responsibility** to edit the `.xacro` model description and put the corresponding parameters to the `spawner_params`.
