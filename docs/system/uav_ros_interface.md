---
layout: default
title: UAV-ROS interface
parent: The UAV system
nav_order: 3
---

# UAV-ROS interface

The UAV can be given commands via the following [managers](https://github.com/ctu-mrs/mrs_uav_managers) and the [state estimator](https://github.com/ctu-mrs/mrs_uav_odometry#mrs-uav-odometry-).

## ControlManager

The [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#ControlManager) takes care of executing the [trackers](https://github.com/ctu-mrs/mrs_uav_trackers#mrs-uav-trackers-) and [controllers](https://github.com/ctu-mrs/mrs_uav_controllers#mrs-uav-controllers-) and it maintains one of each as an *active*.
The controllers handle a feedback loop for stabilization and control of the UAV.
The trackers are the reference generators for the controllers.
High-level navigation (or a user) does not interact with the controllers/trackers directly.
The *managers* provide most of the interface*.

### The flow of information

The ControManager is subscribed to a source of [UAV odometry](https://github.com/ctu-mrs/mrs_uav_odometry#mrs-uav-odometry-), and it hands it to the trackers and controllers.
With each update of the state estimate, the currently active tracker produces a new reference, and the currently active controller generates a new control command.
The controllers and trackers can be switched in mid-flight to accommodate for mission different scenarios.
Users supply the desired references to the ControlManager, which forwards them to the currently active tracker and controller.

### Provided topics

Please refere to [control_manager.launch](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/launch/control_manager.launch) for a complete list of topics.
Notable topics:

General topics reporting on the current state of the ControlManager:

| **topic**                           | **description**                           | **topic type**                                                                                              |
|-------------------------------------|-------------------------------------------|-------------------------------------------------------------------------------------------------------------|
| control_manager/diagnostics         | status of the ControlManager              | [mrs_msgs/ControlManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/ControlManagerDiagnostics.html) |
| control_manager/position_cmd        | control reference from the active tracker | [mrs_msgs/PositionCommand](https://ctu-mrs.github.io/mrs_msgs/msg/PositionCommand.html)                     |
| control_manager/attitude_cmd        | output from the active controller         | [mrs_msgs/AttitudeCommand](https://ctu-mrs.github.io/mrs_msgs/msg/AttitudeCommand.html)                     |
| control_manager/mass_estimate       | total estimated mass                      | `std_msgs/Float64`                                                                                          |
| control_manager/current_constraints | current values of the dynamic constraints | [mrs_msgs::TrackerConstraints](https://ctu-mrs.github.io/mrs_msgs/msg/TrackerConstraints.html)              |
| control_manager/heading             | current heading                           | [mrs_msgs::Float64Stamped](https://ctu-mrs.github.io/mrs_msgs/msg/Float64Stamped.html)                      |

Topics dedicated to Rviz vizualization:

| **topic**                                   | **description**                                         | **topic type**                    |
|---------------------------------------------|---------------------------------------------------------|-----------------------------------|
| control_manager/cmd_odom                    | control reference from the active tracker (Rviz-able)   | `nav_msgs/Odometry`               |
| control_manager/safety_area_markers         | Rviz markers showing the safety area                    | `visualization_msgs::MarkerArray` |
| control_manager/safety_area_coordinates     | Rviz markers showing the coordinates of the safety area | `visualization_msgs::MarkerArray` |
| control_manager/disturbance_markers         | Rviz markers showing the estimated distrubances         | `visualization_msgs::MarkerArray` |
| control_manager/trajectory_original/poses   | pose array re-publishing the original set trajectory    | `geometry_msgs::PoseArray`        |
| control_manager/trajectory_original/markers | Rviz markers re-publishing the original set trajectory  | `visualization_msgs::MarkerArray` |

### Selected services for human-to-machine interaction:

Press `<tab>` after typing in the desired service to auto-complete the default arguments.

Use those services to interact with the done from the terminal:

| **service**                          | **description**                                   | **service type**                                                  | **args**      |
|--------------------------------------|---------------------------------------------------|-------------------------------------------------------------------|---------------|
| control_manager/goto                 | fly to given coordinates                          | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_fcu             | fly to giv en coordinates in the drone's frame    | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_relative        | fly to relative coordinates in the world frame    | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_altitude        | fly to a given height/altitude (the z coordinate) | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[z]`         |
| control_manager/set_heading          | set the heading                                   | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[hdg]`       |
| control_manager/set_heading_relative | set a relative heading                            | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[hdg]`       |

However, these services should not be used from within a program, since they lack the *Header*.
The message *header* contains the frame of reference name and the timestamp.

### Selected services for program-to-machine interaction:

Setting references:

| **service**                          | **description**                           | **service type**                                                                                      |
|--------------------------------------|-------------------------------------------|-------------------------------------------------------------------------------------------------------|
| control_manager/reference            | fly to given coordinates in a given frame | [mrs_msgs/ReferenceSrv](https://ctu-mrs.github.io/mrs_msgs/srv/ReferenceSrv.html)                     |
| control_manager/trajectory_reference | fly along a given trajectory              | [mrs_msgs/TrajectoryReferenceSrv](https://ctu-mrs.github.io/mrs_msgs/srv/TrajectoryReferenceSrv.html) |

Control of trajectory tracking:

| **service**                                | **description**                               | **service type**   |
|--------------------------------------------|-----------------------------------------------|--------------------|
| control_manager/goto_trajectory_start      | fly to the first point in the trajectory      | `std_srvs/Trigger` |
| control_manager/start_trajectory_tracking  | start trajectory tracking                     | `std_srvs/Trigger` |
| control_manager/stop_trajectory_tracking   | stop trajectory tracking                      | `std_srvs/Trigger` |
| control_manager/resume_trajectory_tracking | resume previously stopped trajectory tracking | `std_srvs/Trigger` |

Control of trackers and controllers:

| **service**                       | **description**     | **service type**                                                      |
|-----------------------------------|---------------------|-----------------------------------------------------------------------|
| control_manager/switch_tracker    | switch a tracker    | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |
| control_manager/switch_controller | switch a controller | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

Safety and higher-level flight control:

| **service**                         | **description**                                              | **service type**   |
|-------------------------------------|--------------------------------------------------------------|--------------------|
| control_manager/ehover              | emergency hover                                              | `std_srvs/Trigger` |
| control_manager/eland               | emergency landing                                            | `std_srvs/Trigger` |
| control_manager/failsafe            | emergency failsafe landing                                   | `std_srvs/Trigger` |
| control_manager/failsafe_escalating | emergency escalating failsafe                                | `std_srvs/Trigger` |
| control_manager/motors              | activates/deactivates the control output (don't touch this!) | `std_srvs/SetBool` |

## UavManager

The [UavManager](https://github.com/ctu-mrs/mrs_uav_managers#UavManager) handles higher-level routines such as takeoff and landing.
It also carries some non-essential safety routines.

### Provided services

| **service**            | **description**                            | **service type**                                                                          |
|------------------------|--------------------------------------------|-------------------------------------------------------------------------------------------|
| uav_manager/takeoff    | take off                                   | `std_srvs/Trigger`                                                                        |
| uav_manager/land       | land                                       | `std_srvs/Trigger`                                                                        |
| uav_manager/land_home  | return to the takeoff coordinates and land | `std_srvs/Trigger`                                                                        |
| uav_manager/land_there | land on a particular place                 | [mrs_msgs/ReferenceStamped](https://ctu-mrs.github.io/mrs_msgs/msg/ReferenceStamped.html) |

## ConstraintManager

The [ConstraintManager](https://github.com/ctu-mrs/mrs_uav_managers#ConstraintManager) handles the definition and switching of dynamics constraints for trackers.
The constraints are supplied to all the loaded trackers by the ControlManager.
To simplify the system structure, the ConstraintManager was created to load user-defined constraints from parameter files.
The ConstraintManager maintains feasible constraints active during the flight based on the currently active [odometry source](https://github.com/ctu-mrs/mrs_uav_odometry#mrs-uav-odometry-) and allows users to change them by ROS service.

### Provided topics

| **topic**                      | **description**                 | **topic type**                                                                                                    |
|--------------------------------|---------------------------------|-------------------------------------------------------------------------------------------------------------------|
| constraint_manager/diagnostics | status of the ConstraintManager | [mrs_msgs/ConstraintManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/ConstraintManagerDiagnostics.html) |

### Provided services

| **service**                        | **description**                  | **service type**                                                      |
|------------------------------------|----------------------------------|-----------------------------------------------------------------------|
| constraint_manager/set_constraints | activate the desired constraints | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

## GainManager

The [GainManager](https://github.com/ctu-mrs/mrs_uav_managers#GainManager) handles the definition and switching of controller gains for the [So3Controller](https://github.com/ctu-mrs/mrs_uav_controllers#available-controllers).
The So3Controller should be tuned for a particular UAV model, desired dynamics, and the [Odometry](https://github.com/ctu-mrs/mrs_uav_odometry#mrs-uav-odometry-) under which it is going to fly.
A proper set of gains needs to be provided based on the flight conditions and odometry source.

### Provided topics

| **topic**                | **description**           | **topic type**                                                                                        |
|--------------------------|---------------------------|-------------------------------------------------------------------------------------------------------|
| gain_manager/diagnostics | status of the GainManager | [mrs_msgs/GainManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/GainManagerDiagnostics.html) |

### Provided services

| **service**            | **description**            | **service type**                                                      |
|------------------------|----------------------------|-----------------------------------------------------------------------|
| gain_manager/set_gains | activate the desired gains | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

## MRS Odometry


The [mrs_uav_odometry](https://github.com/ctu-mrs/mrs_uav_odometry#mrs-uav-odometry-) handles the fusion of various measurements of the UAV state variables (position, velocity, acceleration, heading, heading rate) and publishes odometry messages for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#ControlManager).
The estimation of UAV odometry is seperated into lateral, altitude and heading estimation based on sets of three-state Linear Kalman Filters to keep the models simple and modular.

### Provided topics

Please refer to [odometry.launch](https://github.com/ctu-mrs/mrs_uav_odometry/blob/master/launch/odometry.launch) for a complete list of topics.

Notable topics:

| **topic**               | **description**                                                                       | **topic type**                                                            |
|-------------------------|---------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| odometry/uav_state      | UAV state msg used for control                                                        | [mrs_msgs/UavState](https://ctu-mrs.github.io/mrs_msgs/msg/UavState.html) |
| odometry/odom_main      | Odometry msg used for control                                                         | `nav_msgs/Odometry`                                                       |
| odometry/slow_odom      | Odometry msg published at lower rate (2 Hz default)                                   | `nav_msgs/Odometry`                                                       |
| odometry/odom_local     | Odometry msg with origin in the takeoff position                                      | `nav_msgs/Odometry`                                                       |
| odometry/odom_stable    | Odometry msg without discontinuities caused by estimator switches                     | `nav_msgs/Odometry`                                                       |
| odometry/rtk_local_odom | Unfiltered RTK odometry msg. Unsuitable for control, but can be used as ground truth. | `nav_msgs/Odometry`                                                       |

### Provided services

| **service**                               | **description**                                         | **service type**                                                      |
|-------------------------------------------|---------------------------------------------------------|-----------------------------------------------------------------------|
| odometry/toggle_garmin                    | turns Garmin rangefinder correction on/off              | `std_srvs::SetBool`                                                   |
| odometry/change_odometry_source           | start using the requested odometry source for control   | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |
| odometry/change_estimator_type_string     | start using the requested lateral estimator for control | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |
| odometry/change_alt_estimator_type_string | start using the requested altitude source for control   | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |
| odometry/change_hdg_estimator_type_string | start using the requested heading source for control    | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

