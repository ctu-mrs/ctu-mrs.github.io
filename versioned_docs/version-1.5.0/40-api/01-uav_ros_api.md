---
title: UAV-ROS API
pagination_label: ROS API for the MRS System
description: ROS API for the MRS System
---

# MRS System's UAV-ROS API

The UAV can be given commands via the following [managers](https://github.com/ctu-mrs/mrs_uav_managers) and nodes.

## ControlManager

The [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#ControlManager) takes care of executing the [trackers](https://github.com/ctu-mrs/mrs_uav_trackers#mrs-uav-trackers-) and [controllers](https://github.com/ctu-mrs/mrs_uav_controllers#mrs-uav-controllers-) and it maintains one of each as an *active* one.
The controllers handle a feedback loop for stabilization and control of the UAV.
The trackers are the reference generators for the controllers.
High-level navigation (or a user) does not interact with the controllers/trackers directly.
The *managers* provide most of the interface*.

### The flow of information

The ControlManager is subscribed to a source of [UAV State](https://ctu-mrs.github.io/mrs_msgs/msg/UavState.html), and it hands it to the trackers and controllers.
With each update of the UAV State, the currently active tracker produces a new reference, and the currently active controller generates a new control output.
The controllers and trackers can be switched in mid-flight to accommodate for different mission scenarios.
Users supply the desired references to the ControlManager, which forwards the references to the currently active tracker and controller.

### Provided topics

Please refer to [control_manager.launch](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/launch/control_manager.launch) for a complete list of topics.
Notable topics:

General topics reporting on the current state of the ControlManager:

| **topic**                           | **description**                           | **topic type**                                                                                              |
|-------------------------------------|-------------------------------------------|-------------------------------------------------------------------------------------------------------------|
| control_manager/diagnostics         | status of the ControlManager              | [mrs_msgs/ControlManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/ControlManagerDiagnostics.html) |
| control_manager/tracker_cmd         | control reference from the active tracker | [mrs_msgs/TrackerCommand](https://ctu-mrs.github.io/mrs_msgs/msg/TrackerCommand.html)                       |
| control_manager/mass_estimate       | total estimated mass                      | `std_msgs/Float64`                                                                                          |
| control_manager/current_constraints | current values of the dynamic constraints | [mrs_msgs::DynamicsConstraints](https://ctu-mrs.github.io/mrs_msgs/msg/DynamicsConstraints.html)            |
| control_manager/heading             | current heading                           | [mrs_msgs::Float64Stamped](https://ctu-mrs.github.io/mrs_msgs/msg/Float64Stamped.html)                      |

Topics dedicated to Rviz visualization:

| **topic**                                   | **description**                                         | **topic type**                    |
|---------------------------------------------|---------------------------------------------------------|-----------------------------------|
| control_manager/control_reference           | control reference from the active tracker               | `nav_msgs/Odometry`               |
| control_manager/safety_area_markers         | Rviz markers showing the safety area                    | `visualization_msgs::MarkerArray` |
| control_manager/safety_area_coordinates     | Rviz markers showing the coordinates of the safety area | `visualization_msgs::MarkerArray` |
| control_manager/disturbance_markers         | Rviz markers showing the estimated disturbances         | `visualization_msgs::MarkerArray` |
| control_manager/trajectory_original/poses   | pose array re-publishing the original set trajectory    | `geometry_msgs::PoseArray`        |
| control_manager/trajectory_original/markers | Rviz markers re-publishing the original set trajectory  | `visualization_msgs::MarkerArray` |

### Selected services for human-to-machine interaction:

Press `<tab>` after typing in the desired service to auto-complete the default arguments.

Use those services to interact with the drone from the terminal:

| **service**                          | **description**                                   | **service type**                                                  | **args**      |
|--------------------------------------|---------------------------------------------------|-------------------------------------------------------------------|---------------|
| control_manager/goto                 | fly to given coordinates                          | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_fcu             | fly to given coordinates in the drone's frame     | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_relative        | fly to relative coordinates in the world frame    | [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) | `[x,y,z,hdg]` |
| control_manager/goto_altitude        | fly to a given height/altitude (the z coordinate) | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[z]`         |
| control_manager/set_heading          | set the heading                                   | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[hdg]`       |
| control_manager/set_heading_relative | set a relative heading                            | [mrs_msgs/Vec1](https://ctu-mrs.github.io/mrs_msgs/srv/Vec1.html) | `[hdg]`       |

However, these services should not be used from within a program, since they lack the *Header*.
The message *header* contains the frame of reference name and the timestamp.

### Selected services for program-to-machine interaction:

Setting references:

| **service**                          | **description**                                  | **service type**                                                                                                |
|--------------------------------------|--------------------------------------------------|-----------------------------------------------------------------------------------------------------------------|
| control_manager/reference            | fly to given coordinates in a given frame        | [mrs_msgs/ReferenceStampedSrv](https://ctu-mrs.github.io/mrs_msgs/srv/ReferenceStampedSrv.html)                 |
| control_manager/velocity_reference   | fly using velocity command (requires MpcTracker) | [mrs_msgs/VelocityReferenceStampedSrv](https://ctu-mrs.github.io/mrs_msgs/srv/VelocityReferenceStampedSrv.html) |
| control_manager/trajectory_reference | fly along a given trajectory                     | [mrs_msgs/TrajectoryReferenceSrv](https://ctu-mrs.github.io/mrs_msgs/srv/TrajectoryReferenceSrv.html)           |

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
| control_manager/hover               | hover (cancels out the previous command and stops the UAV)   | `std_srvs/Trigger` |
| control_manager/ehover              | emergency hover (=hover, + disables future inputs)           | `std_srvs/Trigger` |
| control_manager/eland               | emergency landing, disables user's input                     | `std_srvs/Trigger` |
| control_manager/failsafe            | emergency failsafe landing                                   | `std_srvs/Trigger` |
| control_manager/failsafe_escalating | emergency escalating failsafe                                | `std_srvs/Trigger` |
| control_manager/motors              | activates/deactivates the control output (don't touch this!) | `std_srvs/SetBool` |

Transformation services:

| **service**                         | **description**                                                          | **service type**                                                                                    |
|-------------------------------------|--------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------|
| control_manager/transform_pose      | uses the mrs_lib::Transformer to transform geometry_msgs::PoseStamped    | [mrs_msgs/TransformPoseSrv](https://ctu-mrs.github.io/mrs_msgs/srv/TransformPoseSrv.html)           |
| control_manager/transform_vector    | uses the mrs_lib::Transformer to transform geometry_msgs::Vector3Stamped | [mrs_msgs/TransformVector3Srv](https://ctu-mrs.github.io/mrs_msgs/srv/TransformVector3Srv.html)     |
| control_manager/transform_reference | uses the mrs_lib::Transformer to transform mrs_msgs::ReferenceStamped    | [mrs_msgs/TransformReferenceSrv](https://ctu-mrs.github.io/mrs_msgs/srv/TransformReferenceSrv.html) |

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
The ConstraintManager maintains feasible constraints active during the flight based on the currently active estimator and allows users to change them by ROS service.

### Provided topics

| **topic**                      | **description**                 | **topic type**                                                                                                    |
|--------------------------------|---------------------------------|-------------------------------------------------------------------------------------------------------------------|
| constraint_manager/diagnostics | status of the ConstraintManager | [mrs_msgs/ConstraintManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/ConstraintManagerDiagnostics.html) |

### Provided services

| **service**                        | **description**                  | **service type**                                                      |
|------------------------------------|----------------------------------|-----------------------------------------------------------------------|
| constraint_manager/set_constraints | activate the desired constraints | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

## GainManager

The [GainManager](https://github.com/ctu-mrs/mrs_uav_managers#GainManager) handles the definition and switching of controller gains for the [Se3Controller](https://github.com/ctu-mrs/mrs_uav_controllers#available-controllers).
The Se3Controller should be tuned for a particular UAV model, desired dynamics, and the state estimator used for obtained the UAV State.
A proper set of gains needs to be provided based on the flight conditions and odometry source.

### Provided topics

| **topic**                | **description**           | **topic type**                                                                                        |
|--------------------------|---------------------------|-------------------------------------------------------------------------------------------------------|
| gain_manager/diagnostics | status of the GainManager | [mrs_msgs/GainManagerDiagnostics](https://ctu-mrs.github.io/mrs_msgs/msg/GainManagerDiagnostics.html) |

### Provided services

| **service**            | **description**            | **service type**                                                      |
|------------------------|----------------------------|-----------------------------------------------------------------------|
| gain_manager/set_gains | activate the desired gains | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

## Estimation Manager

The [EstimationManager](https://github.com/ctu-mrs/mrs_uav_managers#EstimationManager) handles the fusion of various measurements of the UAV state variables (position, velocity, acceleration, heading, heading rate) and publishes [UAV State](https://ctu-mrs.github.io/mrs_msgs/msg/UavState.html) messages for the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#ControlManager).

### Provided topics

Please refer to [estimation_manager.launch](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/launch/estimation_manager.launch) for a complete list of topics.

Notable topics:

| **topic**                     | **description**                          | **topic type**                                                                         |
|-------------------------------|------------------------------------------|----------------------------------------------------------------------------------------|
| estimation_manager/uav_state  | UAV state msg used for control           | [mrs_msgs/UavState](https://ctu-mrs.github.io/mrs_msgs/msg/UavState.html)              |
| estimation_manager/odom_main  | current UAV state in the _Odometry_ type | `nav_msgs/Odometry`                                                                    |
| estimation_manager/height_agl | height above ground level                | [mrs_msgs::Float64Stamped](https://ctu-mrs.github.io/mrs_msgs/msg/Float64Stamped.html) |

### Provided services

| **service**                         | **description**               | **service type**                                                      |
|-------------------------------------|-------------------------------|-----------------------------------------------------------------------|
| estimation_manager/change_estimator | switch to a desired estimator | [mrs_msgs/String](https://ctu-mrs.github.io/mrs_msgs/srv/String.html) |

## Transform manager

The [TransformManager](https://github.com/ctu-mrs/mrs_uav_managers#TransformManager) handles the broadcasting of default TFs.
Can construct custom [configurable](https://github.com/ctu-mrs/mrs_uav_managers/blob/f9e0eac0680cc1cd2f5bea0e2d55199c8af1178a/config/public/transform_manager/transform_manager.yaml#L53C6-L53C6) TFs from `nav_msgs/Odometry` topics by adding them to the `tf_sources` [array](https://github.com/ctu-mrs/mrs_uav_managers/blob/f9e0eac0680cc1cd2f5bea0e2d55199c8af1178a/config/public/transform_manager/transform_manager.yaml#L48C11-L48C11) in custom config.
Can republish a `nav_msgs/Odometry` topic in another frame by adding the `frame_id` to the `republish_in_frames` [array](https://github.com/ctu-mrs/mrs_uav_managers/blob/f9e0eac0680cc1cd2f5bea0e2d55199c8af1178a/config/public/transform_manager/transform_manager.yaml#L61).

### Provided default TFs

| **TF**              | **description**                                                                                                                                | **example usage**                                                                       |
|---------------------|------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| fcu_untilted_origin | (heading only) fcu frame with removed tilts (z-axis is aligned with the gravity vector)                                                        | commanding UAV in body frame disregarding tilts                                         |
| world_origin        | origin position defined by the world file, ENU orientation, based on UTM estimators (GPS, RTK, etc.), does not exist for non-UTM estimators    | commanding UAV localized by UTM-based estimator in a locally defined coordinate world   |
| local_origin        | origin pose defined by the initial pose of the UAV (including orientation), exists for all estimators                                          | commanding UAV w.r.t. initial pose, universal frame that is always available            |
| fixed_origin        | origin pose defined by the initial estimator, origin does not move after estimator switch, odometry in this frame jumps after estimator switch | commanding UAV w.r.t. initial estimator frame, universal frame that is always available |
| stable_origin       | origin pose defined by the initial estimator, origin jumps after estimator switch, odometry in this frame is smooth without jumps              | mapping position of detected objects invariant to estimator switching                   |
| utm_origin          | origin position defined by the intersetion of the equator and the zone's central meridian, ENU orientation                                     | commanding UAV in absolute metric coordinates irregardless of the current world file    |
| mapping_origin_tf   | origin position and heading defined by initialization of a SLAM algorithm, origin tilt around x and y axes optionally defined by custom topic  | mapping of environment using a SLAM that cannot estimate orientation                    |


## Trajectory generation

The [mrs_uav_trajectory_generation](https://github.com/ctu-mrs/mrs_uav_trajectory_generation) serves to generate a feasible time-parametrized trajectory from a desired waypoint path.

### Provided topics

| **topic**                  | **description**     | **topic type**                                                    |
|----------------------------|---------------------|-------------------------------------------------------------------|
| trajectory_generation/path | desired path to fly | [mrs_msgs/Path](https://ctu-mrs.github.io/mrs_msgs/msg/Path.html) |

### Provided services

| **service**                | **description**     | **service type**                                                        |
|----------------------------|---------------------|-------------------------------------------------------------------------|
| trajectory_generation/path | desired path to fly | [mrs_msgs/PathSrv](https://ctu-mrs.github.io/mrs_msgs/srv/PathSrv.html) |
