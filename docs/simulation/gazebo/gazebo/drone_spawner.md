---
layout: default
title: MRS Drone Spawner
parent: Gazebo
grand_parent: Simulation
nav_order: 2
---

| :warning: **Attention please: The software is still in development phase.**                                          |
| :---                                                                                                                 |
| Please keep in mind that spawner update is backwards-incompatible and some aspects of the workflow may have changed. |

# About

The `mrs_drone_spawner` is a ROS node written in Python, which allows you to dynamically add drones into your Gazebo simulation.
We have also developed a **Spawner API** to facilitate the development of new modular Gazebo models derived from the MRS UAV System.
At the core, the spawner handles the following operations:

* generating an SDF model from the [available templates](https://github.com/ctu-mrs/mrs_uav_system#unmanned-aerial-vehicles)
* adding optional components (sensors, plugins...) based on the user input
* running the [PX4 Firmware SITL](https://dev.px4.io/master/en/simulation/ros_interface.html) (software-in-the-loop),
* running [MAVROS](http://wiki.ros.org/mavros), which handles the communication (MAVLink) between the firmware and the simulator.

# Spawning a drone

The `mrs_drone_spawner` is **started automatically** with the simulator when starting Gazebo with our launch file
```bash
roslaunch mrs_uav_gazebo_simulation simulation.launch
```

If you use a custom launch file to start Gazebo, you may need to start the spawner node **manually** by calling
```bash
roslaunch mrs_uav_gazebo_simulation mrs_drone_spawner.launch
```

To add a vehicle into the simulation, call a service provided by the drone spawner
```bash
rosservice call /mrs_drone_spawner/spawn "1 --x500 --enable-rangefinder"
```

The service takes a **string** as an argument.
Use the string to pass arguments to the spawner and define the vehicle configuration.
The string should contain the following:

* `device IDs` - non-negative integers, separated by spaces. These will be auto-assigned if no ID is specified.
* `model` - use `--` and a model name to select a specific model (e.g. `--x500` to use the template ). The model name should match the name of a jinja template file without suffix.
* `keywords` - specified inside jinja macros as a `spawner_keyword`. Add `--` before each keyword (e.g. `--enable-rangefinder` to add a Garmin rangefinder).
* `component args` - after a keyword, you can specify arguments for that component. Use ":=" to assign a new argument value (e.g. `--enable-ground-truth update_rate:=50` to set the update rate of the ground truth plugin to 50 Hz). Use **Python-like syntax for boolean arguments** (`True` or `False` with capital letters).

This example will spawn a single drone with an ID 3, set the vehicle type to x500, add a Garmin rangefinder, and an Ouster OS0-128 3D LiDAR. The command will also modify some of the default LiDAR params (set horizontal samples to 128, set update rate to 8 Hz):
```bash
rosservice call /mrs_drone_spawner/spawn "3 --x500 --enable-rangefinder --enable-ouster model:=OS0-128 use_gpu:=True horizontal_samples:=128 update_rate:=8"
```

**NOTE**: The rosservice CLI may not allow the string to start with a dash.
Add a space at the beginning of the string to prevent errors, like in the example below.

This example will spawn a single drone of the type f550, and add a Garmin rangefinder. The vehicle ID will be automatically assigned as the lowest unused positive integer.
```bash
rosservice call /mrs_drone_spawner/spawn " --f550 --enable-rangefinder"
```


## Spawning multiple drones with the same configuration (e.g. for swarms)
For simulations with multiple drones using identical configuration, we provide a convenient way to spawn an entire swarm with just one command.

This example will spawn 5 drones, set the vehicle type to f450, add a Garmin rangefinder, blinking UV LEDs below the propellers, and 2 wide-angle cameras sensitive to UV light.
```bash
rosservice call /mrs_drone_spawner/spawn "1 2 3 4 5 --f450 --enable-rangefinder --enable-uv-leds --enable-dual-uv-cameras"
```

Note that this may take a while, as each model has to be added into the Gazebo world, PX4 SITL and MAVROS have to be launched and mavlink ports have to be connected for each vehicle separately.

## Help

There are two modes of displaying help: general and model-specific.

To display a general help for the spawner, call the service with `--help`.
NOTE: a space needs to be added before the dash, otherwise rosservice CLI will not parse your input.
```bash
rosservice call /mrs_drone_spawner/spawn " --help"
```

To display a model-specific help, add a model name to the arguments:
```bash
rosservice call /mrs_drone_spawner/spawn " --x500 --help"
```
This will display a list of keywords that are supported by the selected model, together with the description of individual components and the component arguments.

**NOTE:** The help text returned by the service call is unfortunately not nicely formatted in console.
Look for a readable help in the the window where simulation/drone_spawner is running.

## Naming and spawn poses

The spawner will assign the name `uav` + ID to each new model.
The default naming can be changed in the **config file** (param `gazebo_models/default_robot_name`) or with a param `--name`.

Example: Spawn a single t650 drone with an auto-assigned ID (lowest unassigned integer between 0 and 255).
The vehicle will be named `pes` + ID.
```bash
rosservice call /mrs_drone_spawner/spawn " --t650 --name pes"
```

New vehicles are placed in randomly assigned positions with a random heading.
To prevent collisions, the spawn pose is sampled from a circle with a radius defined in the **config file** (param `gazebo_models/spacing`).

Vehicles can be spawned at exact positions by using the param `--pos` followed by exactly 4 numbers (x, y, z, heading).

Example: spawn a single x500 drone with ID 1, place it at coords [30, 0, 0] with an initial heading of 1.57 radians.
```bash
rosservice call /mrs_drone_spawner/spawn "1 --x500 --pos 30 0 0 1.57"
```

It is also possible to specify the spawn poses in a separate file (useful for multiple vehicles).
Use the param `--pos-file` followed by a filepath to the config file.
The file may be a `.yaml` or a `.csv`.

An example of a yaml pos-file:
```yaml
uav1:
  id: 1
  x: 10
  y: 2.5
  z: 0.5
  heading: -0.3

uav2:
  id: 2
  x: 22
  y: 14
  z: 0.5
  heading: 3.14
```

An example of a csv pos-file:
```yaml
1, 10, 2.5, 0.5, -0.3
2, 22, 14, 0.5, 3.14
```

## Mavlink GCS connection

By default, the UAV will connect to Gazebo using Mavlink on port 14550. If you want to use QGroundControl or access raw Mavlink data, you can use the spawner param `--enable_mavlink_gcs` to add an extra mavlink stream.
Note that this will not cause the drone to autoconnect to QGC, but you can manually connect in Application Settings -> Comm Links -> Add -> Type: UDP -> port: MAVLINK_GCS_PORT_REMOTE.
The extra stream is configured [here](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink_gcs) and the UDP ports will be automatically assigned according to the UAV's ID as a `gcs_base_port + ID`. The stream will listen to the simulation data at a MAVLINK_GCS_UDP_PORT_LOCAL (default `18000 + ID`) and publish on a MAVLINK_GCS_UDP_PORT_REMOTE (default `18100 + ID`).
Default port nubmers are loaded from the config file [spawner_params.yaml](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/config/spawner_params.yaml).

## Jinja templates and SDF models

We only provide model templates in Jinja format (suffix `.sdf.jinja`), which need to be *rendered*.
The rendered SDF models are loaded direcly into Gazebo by the spawner.
The SDF models may also be stored in temporary files if enabled in the **config file** (param `jinja_templates/save_rendered_sdf`).
If enabled, the temporary files are saved to `/tmp` and named `mrs_drone_spawner_` + timestamp + unique hash + model type + vehicle name + `.sdf`.

## Extra resources and external paths

By default, the Jinja environment will load the [models](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/tree/master/ros_packages/mrs_uav_gazebo_simulation/models) folder and look for jinja templates with suffix matching the definition in the **config file** (param `jinja_templates/suffix`).
Extra resources may be loaded by adding filepaths to the param `extra_resource_paths` in the **config file**.
It is highly encouraged to create new models in an external repository and add this repository to the `extra_resource_paths`.

## Changing the config file
To override the default values in the config file, the launch files `mrs_uav_gazebo_simulation simulation.launch` and `mrs_uav_gazebo_simulation mrs_drone_spawner.launch` use the argument `spawner_config`.

```bash
roslaunch mrs_uav_gazebo_simulation simulation.launch spawner_config:=PATH_TO_MY_CONFIG_FILE
```

# Custom drone development

This section is dedicated to all ambitious drone developers who wish to create their own modular robots for Gazebo.
We have designed a simple Spawner API to facilitate the creation of new platforms and individual components.

## New components
The Spawner API requires each component to be defined as a **Jinja macro**.
Inside the macro, the following variables must be declared:

* `spawner_keyword` - this will become the argument to activate the component through the spawn service call
* `spawner_description` - a descripton of the component displayed as the help
* `spawner_default_args` - `dict` or `none`. Defines the default values of internal parameters that can be overriden from the spawn service call. For example a camera resolution or a plugin update rate.

We provide a generic macro to override the default args with the user input in `generic_components.handle_spawner_args`.
An example of a component that is always active and the spawner only modifies its properties: `propellers_macro` in [component_snippets.sdf.jinja](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/models/mrs_robots_description/sdf/component_snippets.sdf.jinja).

## New platforms

A new platform should always define the meshes, motors and physics properties.
The platform should then only call the macros of additional components.
We provide several quadrotor templates (f330, f450, t650, x500), a 6-rotor (f550) and a coaxial 8-rotor (naki) as examples.
Some parts of the airframe may be conditional, such as holders for specific sensors that depend on the existence of multiple drones.
This can be done by storing the result of a component macro call in a variable and checking its length.
A good example for this is the `lidar_mount` in the [x500.sdf.jinja](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/models/mrs_robots_description/sdf/x500.sdf.jinja) template.
A completely new platform may be created in an external package.
Please refer to the [mrs_gazebo_custom_drone_example](https://github.com/ctu-mrs/mrs_gazebo_custom_drone_example) repository for more information and a [tutorial](https://ctu-mrs.github.io/docs/simulation/gazebo/gazebo/custom_drone.html).

## Jinja syntax

Jinja is a templating language that allows the use of Python-esque syntax inside xml files.
After rendering a template into an SDF model, all Jinja syntax is removed.
Please refer to the project website for Jinja [documentation](https://jinja.palletsprojects.com/en/3.1.x/templates/#synopsis).

## Design philosophy

Two general-purpose templates are used for building blocks shared by multiple robots.
Generic items (a colored mesh, a Gazebo camera plugin...), are defined in `generic_components`.
Components that are compatible with the Spawner API are defined in `component_snippets`.
Platform templates should not implement any components that may be used by other robots.
Use `import` statements to include other templates, the syntax is similar to Python.
