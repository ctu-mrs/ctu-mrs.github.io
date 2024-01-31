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
* `component args` - after a keyword, you can specify arguments for that component. Use ":=" to assign a new argument value (e.g. `--enable-ground-truth update_rate:=50` to set the update rate of the ground truth plugin to 50 Hz).

**NOTE**: The rosservice CLI may not allow the string to start with a dash.
Add a space at the beginning to prevent errors.

Spawning multiple drones with the same configuration (useful for swarms).
This example will spawn 5 drones, set the vehicle type to f450, add a Garmin rangefinder, blinking UV LEDs below the propellers, and 2 wide-angle cameras sensitive to UV light.
```bash
rosservice call /mrs_drone_spawner/spawn "1 2 3 4 5 --f450 --enable-rangefinder --enable-uv-leds --enable-dual-uv-cameras"
```

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

## Jinja syntax

Jinja is a templating language that allows the use of Python-esque syntax inside xml files.
After rendering a template into an SDF model, all Jinja syntax is removed.

* Jinja statemets begin with `\{\%` and end with `\%\}`. This will add an empty line into the rendered file. Using `\{\%-` and `-\%\}` will also remove the empty line and is recommended when contributing to the MRS UAV System.
* Jinja comments begin with `{#` and end with `#}`. No extra line removal is necessary here.
* Jinja macros function similarly to python functions but have to be ended with `endmacro`. Declare a macro with `\{\%- macro my_macro(arg1, arg2, ...) -\%\}` and call the macro with `{{ my_macro(arg1, arg2, ...) }}`
* To declare a variable, use `set`, e.g. like this `\{\%- set my_variable = 10.7 -\%\}`
* Jinja supports `int`, `float`, `string`, `list` and `dict` just like Python. None datatype is spelled with lowercase n: `none`
* You can use `if` and `for` in a fashion similar to Python, but the block needs to be ended with `endif` or `endfor`.
* Data filter - if you want to make sure, that a variable is of a specific type, you can use a type filter. For example `\{\%- set my_string = arg1 | string -\%\}` will make sure that the variable `my_string` is only filled with a string. Filtering can also be used to check the length of an iterable datatype, e.g. `\{\%- if my_list | length > 10 -\%\}`

## Design philosophy

Two general-purpose templates are used for building blocks shared by multiple robots.
Generic items (a colored mesh, a Gazebo camera plugin...), are defined in `generic_components`.
Components that are compatible with the Spawner API are defined in `component_snippets`.
Platform templates should not implement any components that may be used by other robots.
Use `import` statements to include other templates, the syntax is similar to Python.
