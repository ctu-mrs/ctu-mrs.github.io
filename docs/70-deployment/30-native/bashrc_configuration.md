---
title: Configuring .bashrc for real UAVs
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# What is the .bashrc?

You can find the `.bashrc` file in the home folder of the UAV's onboard computer, it is a hidden file.
The purpose of the `.bashrc` file is to set up environmental variables, aliases and other settings.
We use it to define the current configuration of the UAV through a set of environmental variables.

## .bashrc for your personal computer

## .bashrc for a REAL UAV

## Environmental variables:

In the `.bashrc` file of a typical UAV which is running the MRS UAV System, you will find a list of environmental variables similar to this:

```bash
export UAV_NAME=uav1
export RUN_TYPE=realworld
export UAV_TYPE=x500
export UAV_MASS=2.0
export WORLD_NAME=temesvar_field
export INITIAL_DISTURBANCE_X=0.0
export INITIAL_DISTURBANCE_Y=0.0
export SENSORS="pixhawk, rtk, garmin_down, realsense_front"
export OLD_PX4_FW=0
```

Let's run through all the important variables:

```bash
export UAV_NAME="uav1"
```
This variable defines the UAV's namespace, all the ROS nodes of the MRS UAV System will run under the namespace `/$UAV_NAME/node_name`.
The `UAV_NAME` should match the `/etc/hostname` of the onboard computer.
We also have a convention in place that the static IP address of the UAV should correspond to the `UAV_NAME`, e.g.
`uav21` will have a static IP of `192.168.69.121`.

```bash
export RUN_TYPE="realworld"
```

Setting `RUN_TYPE` to `realworld` will tell the system that we are **not** in simulation.

```bash
export UAV_TYPE="x500"
```

The `UAV_TYPE` variable used within the tmux sessions to find the correct _platform configuration file_.

```bash
export UAV_MASS="2.0"
```

This is the total takeoff mass (including the battery) of the UAV in `kg`.
It is used as an initial condition for the mass estimator of the UAV, which is in turn used in the feedforward control actions of the controllers.
It is not critical to set this value precisely before every flight, as the mass estimator will correct errors after a few seconds of flying.
If the `UAV_MASS` is different than the real mass, the UAV can behave erraticaly during takeoff, it can jump up quickly or the takeoff can be slow.

```bash
export INITIAL_DISTURBANCE_X="0.0"
export INITIAL_DISTURBANCE_Y="0.0"
```

The UAV can suffer from an intrinsic internal disturbance, like offset center of mass, motor which is not performing as well as the others and so on.
This disturbance will "pull" the UAV in a certain direction.
The MRS UAV System will estimate and compensate for these disturbances.
The `INITIAL_DISTURBANCE` variable serves as the initial condition for the disturbance estimator.
It is defined in `Newtons` of thrust in the `X` or `Y` directions.
You can leave the values at `0.0` and the UAV will estimate the disturbances on its own relatively quickly.
Only when you require a precise and straight takeoff and the best possible performance right after takeoff, you should measure and define the `INITIAL_DISTURBANCE` values.

```bash
export SENSORS="pixhawk, rtk, garmin_down, realsense_front"
```

This variable contains a list of sensors connected to the onboard computer.
It is read by the `mrs_uav_deployment` [sensors.launch](https://github.com/ctu-mrs/mrs_uav_deployment/blob/master/launch/sensors.launch) launch file, which will in turn launch the appropriate ROS drivers for the specified sensors, as well as publish a static tranformation between the UAV's FCU frame and the sensor frame.

```bash
export WORLD_NAME="temesvar_field"
```

This variable is used to specify a [world file](https://github.com/ctu-mrs/mrs_uav_deployment/tree/master/config/worlds) which is used for the flight.
The world file can define the origin of the global frame, the safety area in which flights can be conducted and the minimum and maximum flight heights.
Note that this only makes sense when a global GNSS-based localization like GPS or RTK is used.
If you use a non-global localization (like laser-slam or visual odometry), use the local world file.
The origin of the local frame will be the starting position of the UAV.

```bash
export export OLD_PX4_FW=0
```

This variable is a temporary workaround for flying with both the _new_ version of Pixhawk6 (with PX4 FW version > 1.13.2) and with the old Pixhawk4 (with older PX4 FW versions).
