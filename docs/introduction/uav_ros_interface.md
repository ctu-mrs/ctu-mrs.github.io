---
layout: default
title: UAV ROS interface
parent: Introduction
nav_oder: 99
---

# UAV - ROS interface

The UAV can be interacted with mainly through the following **managers**.

## ControlManager

The ControlManager takes care of running the **trackers** and **controllers** and maintains one of each as an **active**.
The controllers handle a feedback loop for stabilization and control of the UAV.
The trackers are the reference generators for the controllers.
Although the "hard work" during the flight is done by the tracker and the controllers, users do not interact with them directly.
Most of the interface is hidden behind the ControlManager.

The ControlManager is subscribed to a source of Odometry, and it hands it to the trackers and controllers.
With each update of the state estimate, the currently active tracker produces a new reference, and the currently active controller produces a new control command.
The controllers and trackers can be switched in mid-flight.
Users supply the desired references to the ControlManager, which forwards them to the currently active tracker and controller.

### Selected services for human-to-machine interaction:

Press `[TAB]` after typing in the desired service to see the corresponding arguments.

Use those services to interact with the done from the terminal:

| **service**                      | **description**                                   | **service type** | **args**      |
|----------------------------------|---------------------------------------------------|------------------|---------------|
| control_manager/goto             | fly to given coordinates                          | `mrs_msgs/Vec4`  | `[x,y,z,hdg]` |
| control_manager/goto_fcu         | fly to giv en coordinates in the drone's frame    | `mrs_msgs/Vec4`  | `[x,y,z,hdg]` |
| control_manager/goto_relative    | fly to relative coordinates in the world frame    | `mrs_msgs/Vec4`  | `[x,y,z,hdg]` |
| control_manager/goto_altitude    | fly to a given height/altitude (the z coordinate) | `mrs_msgs/Vec1`  | `[z]`         |
| control_manager/set_yaw          | set just the yaw (heading)                        | `mrs_msgs/Vec1`  | `[hdg]`       |
| control_manager/set_yaw_relative | set a relative yaw (heading)                      | `mrs_msgs/Vec1`  | `[hdg]`       |

However, these services should not be used from within a program, since they lack the *Header* which usually contains the frame of reference name and the timestamp.

### Selected services for program-to-machine interaction:

Setting references:

| **service**                          | **description**                           | **service type**                  |
|--------------------------------------|-------------------------------------------|-----------------------------------|
| control_manager/reference            | fly to given coordinates in a given frame | `mrs_msgs/ReferenceStampedSrv`    |
| control_manager/trajectory_reference | fly along a given trajectory              | `mrs_msgs/TrajectoryReferenceSrv` |

Control of trajectory tracking:

| **service**                                | **description**                               | **service type**   |
|--------------------------------------------|-----------------------------------------------|--------------------|
| control_manager/goto_trajectory_start      | fly to the first point in the trajectory      | `std_srvs/Trigger` |
| control_manager/start_trajectory_tracking  | start trajectory tracking                     | `std_srvs/Trigger` |
| control_manager/stop_trajectory_tracking   | stop trajectory tracking                      | `std_srvs/Trigger` |
| control_manager/resume_trajectory_tracking | resume previously stopped trajectory tracking | `std_srvs/Trigger` |

Control of trackers and controllers:

| **service**                       | **description**      | **service type**  |
|-----------------------------------|----------------------|-------------------|
| control_manager/switch_tracker    | switch a tracker     | `mrs_msgs/String` |
| control_manager/switch_controller | switch a  controller | `mrs_msgs/String` |

Safety and higher-level flight control:

| **service**                         | **description**                                              | **service type**   |
|-------------------------------------|--------------------------------------------------------------|--------------------|
| control_manager/ehover              | emergency hover                                              | `std_srvs/Trigger` |
| control_manager/eland               | emergency landing                                            | `std_srvs/Trigger` |
| control_manager/failsafe            | emergency failsafe landing                                   | `std_srvs/Trigger` |
| control_manager/failsafe_escalating | emergency escalating failsafe                                | `std_srvs/Trigger` |
| control_manager/motors              | activates/deactivates the control output (don't touch this!) | `std_srvs/SetBool` |

## UavManager

The UavManager handles higher-level routines such as takeoff and landing.
It also carries some non-essential safety routines.

Provided services:

| **service**           | **description**                      | **service type**   |
|-----------------------|--------------------------------------|--------------------|
| uav_manager/takeoff   | take off                             | `std_srvs/Trigger` |
| uav_manager/land      | land                                 | `std_srvs/Trigger` |
| uav_manager/land_home | return to the takeoff place and land | `std_srvs/Trigger` |

## ConstraintManager

The ConstraintManager handles the definition and switching of dynamics constraints.
The constraints are mutual for all the **trackers** and supplied to them by the **ControlManager**.
However, to simplify the system structure, the ConstraintManager was created to load user-defined constraints from parameter files.
The ConstraintManager maintains feasible constraints active during the flight based on the currently active odometry source and allows to change them by ROS service.

Provided services:

| **service**                        | **description**                  | **service type**  |
|------------------------------------|----------------------------------|-------------------|
| constraint_manager/set_constraints | activate the desired constraints | `mrs_msgs/String` |

## GainManager

The GainManager handles the definition and switching of controller gains for the **So3Controller**.
The **So3Controller**, one of our controllers, is highly tunable.
A proper set of gains needs to be set based on the flight conditions and odometry source.

Provided services:

| **service**            | **description**            | **service type**  |
|------------------------|----------------------------|-------------------|
| gain_manager/set_gains | activate the desired gains | `mrs_msgs/String` |
