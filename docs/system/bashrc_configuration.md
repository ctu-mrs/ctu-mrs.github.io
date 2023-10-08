---
layout: default
title: Configuration of .bashrc for real UAVs
parent: The UAV system
nav_order: 5
---

| :warning: **Attention please: This page is outdated.**                                                                                           |
| :---                                                                                                                                             |
| The MRS UAV System 1.5 is being released and this page needs updating. Plase, keep in mind that the information on this page might not ve valid. |

# What is .basrc?:

You can find the `.bashrc` file in the home folder of the UAV's onboard computer, it is a hidden file.
The purpose of the `.bashrc` file is to set up environmental variables, aliases and other settings.
We use it to define the current configuration of the UAV through a set of environmental variables.

## Environmental variables:

In the `.bashrc` file of a typical UAV which is running the MRS UAV System, you will find a list of environmental variables similar to this:

```bash
export UAV_NAME="uav21" 
export NATO_NAME="oscar" # lower-case name of the UAV frame {alpha, bravo, charlie, ...}
export UAV_MASS="2.9" # [kg], used only with real UAV
export RUN_TYPE="uav" # {simulation, uav}
export UAV_TYPE="x500" # {f550, f450, t650, eagle, naki}
export PROPULSION_TYPE="darpa" # {default, new_esc, ...}
export ODOMETRY_TYPE="gps" # {gps, optflow, hector, vio, ...}
export INITIAL_DISTURBANCE_X="0.2" # [N], external disturbance in the body frame
export INITIAL_DISTURBANCE_Y="-0.1" # [N], external disturbance in the body frame
export STANDALONE="false" # disables the core nodelete manager
export SWAP_GARMINS="false" # swap up/down garmins
export PIXGARM="true" # true if Garmin lidar is connected throught Pixhawk
export SENSORS="garmin_down, realsense_front" # {garmin_down, garmin_up, rplidar, realsense_front, teraranger, bluefox_optflow, realsense_brick, bluefox_brick}
export WORLD_NAME="cisar" # e.g.: "simulation" <= mrs_general/config/world_simulation.yaml
export MRS_STATUS="readme" # {readme, dynamics, balloon, avoidance, control_error, gripper}
export LOGGER_DEBUG="false" # sets the ros console output level to debug
```

Let's run through all the important variables:

```bash
export UAV_NAME="uav21" 
```
This variable defines the UAV's namespace, all the ROS nodes of the MRS UAV System will run under the namespace `/$UAV_NAME/node_name`.
The `UAV_NAME` should match the `/etc/hostname` of the onboard computer.
We also have a convention in place that the static IP address of the UAV should correspond to the `UAV_NAME`, e.g.
`uav21` will have a static IP of `192.168.69.121`.

```bash
export NATO_NAME="oscar" 
```

This is an alternative name of the UAV frame, can be omitted.

```bash
export UAV_MASS="2.9" 
```

This is the total takeoff mass (including the battery) of the UAV in `kg`.
It is used as an initial condition for the mass estimator of the UAV, which is in turn used in the feedforward control actions of the controllers.
It is not critical to set this value precisely before every flight, as the mass estimator will correct errors after a few seconds of flying.
If the `UAV_MASS` is different than the real mass, the UAV can behave erraticaly during takeoff, it can jump up quickly or the takeoff can be slow.

```bash
export RUN_TYPE="uav"
```

Set `RUN_TYPE` to `uav` on real UAVs, set it to `simulation` when you want to run just the simulation.

```bash
export UAV_TYPE="x500"
```

The `UAV_TYPE` variable defines the frame of the UAV, which can be tied to [specific default configs](https://github.com/ctu-mrs/mrs_uav_managers/tree/master/config/uav) for the frame, the propulsion and sensor configurations (explained lower down).

```bash
export PROPULSION_TYPE="darpa"
```

This variable defines the proulsion configuration of the UAV.
The same frame can have different motors and propellers, which will result in different response to throttle commands.
The Pixhawk controller accepts total throttle as an input (0-100%) and we need to corellate this value to the actual physical thrust (and therefore the produced acceleration).
This is done through a quadratic expression that we call motor params.
Motor parameters for specific frames and `PROPULSION_TYPE` are defined in `the mrs_uav_controllers` package.
[Example configuration for the X500 frame is here.](https://github.com/ctu-mrs/mrs_uav_controllers/tree/master/config/uav/x500) 

```bash
export ODOMETRY_TYPE="gps"
```

`ODOMETRY_TYPE` defines the default lateral and altitude estimator which is used, as well as other estimators which should be running.

The value of the variable defines [which odometry config file](https://github.com/ctu-mrs/mrs_uav_odometry/tree/master/config/uav) is used from here.


```bash
export INITIAL_DISTURBANCE_X="0.2"
export INITIAL_DISTURBANCE_Y="-0.1"
```

The UAV can suffer from an intrinsic internal disturbance, like offset center of mass, motor which is not performing as well as the others and so on.
This disturbance will "pull" the UAV in a certain direction.
The MRS UAV System will estimated and compensate for these disturbances.
The `INITIAL_DISTURBANCE` variable serves as the initial condition for the disturbance estimator.
It is defined in `Newtons` of thrust in the `X` or `Y` directions.
You can leave the values at `0.0` and the UAV will estimate the disturbances on its own relatively quickly.
Only when you require a precise and straight takeoff and the best possible performance right after takeoff, you should measure and define the `INITIAL_DITURBANCE` values.

```bash
export STANDALONE="false"
```

The core of the MRS UAV System runs in a single [nodelet](http://wiki.ros.org/nodelet) manager.
If you set `STANDALONE` to `true`, each node will run as a separate node, which can be beneficial for debugging.

```bash
export SWAP_GARMINS="false"
```

This variable was used on UAVs with 2 Garmin rangefinders (one pointing up and one down).
It is deprecated and not used in most of the UAVs.

```bash
export PIXGARM="true"
```

Most of our UAVs have a Garmin Lidar Lite V3 rangefinder.
This rangefinder can be connected to the Pixhawk's I2C port and stream the data to the onboard computer.
If this is the case, set `PIXGARM` to `true`.

```bash
export SENSORS="garmin_down, realsense_front"
```

This variable contains a list of sensors connected to the onboard computer.
It is read by the `mrs_uav_general` [sensors.launch](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/sensors.launch) launch file, which will in turn launch the appropriate ROS drivers for the specified sensors, as well as publish a static tranformation between the UAV's FCU frame and the sensor frame.

```bash
export WORLD_NAME="cisar"
```
This variable is used to specify a [world file](https://github.com/ctu-mrs/mrs_uav_general/tree/master/config/worlds) which is used for the flight.
The world file can define the origin of the global frame, the safety area in which filghts can be conducted and the minimum and maximum flight heights.
Note that this only makes sense when a global localization like gps or rtk is used.
If you use a non-global localization (like laser-slam or optflow), use the local world file.
The origin of the local frame will be the starting position of the UAV.

```bash
export MRS_STATUS="readme"
```

This variable is deprecated and can be omitted.

```bash
export LOGGER_DEBUG="false" #
```

Sets the ros console output level to debug.
