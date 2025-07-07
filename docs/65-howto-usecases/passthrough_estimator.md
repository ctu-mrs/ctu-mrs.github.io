---
title: Passthrough estimator
pagination_label: Using the passthrough estimator within MRS system
description: A guide on how to use the passthrough estimator within MRS system
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Passthrough Estimator

By default, the MRS system uses a state estimator running as a plugin within the `estimation_manager` nodelet to provide the full state estimate required for the feedback control of the UAV.
The state estimator supplies the state estimate at the frequency required for stable flight and verifies whether measurements from all fused sensors/algorithms are coming, have correct timestamp, etc.

Is some cases it might be necessary to bypass the state estimation altogether.
Example use cases include conserving computational power on low-power CPUs, and enabling agile flights that require minimal delay in the estimated state.
In such cases, the passthrough estimator can be used to pass an odometry message from an algorithm into the MRS system.
However, the health checks, filtering, outlier rejection, and other processing steps normally performed by the estimation manager will not occur when the passthrough estimator is used.
Additionally, the `world_origin` frame will not be available while using the passthrough estimator, as it is designed for use both with and without GNSS (which is required to specify `world_origin`).

## ⚠️ Important Warning

**The quality control and sanitization of data when using the passthrough estimator is the responsibility of the user!
Using unhealthy data can be extremely dangerous (e.g., sudden jumps in variables, inconsistent time intervals, velocities in incorrect frames, etc.)!
Be sure to fully understand the implications and always verify functionality in simulation and handheld flight before deployment.**

## Minimal Odometry Quality Checks

The odometry used as input for the passthrough estimator must satisfy at least the following conditions:

* Sufficient Rate — The odometry must be provided at a rate that meets or exceeds the control manager’s requirements for each [control modality](https://github.com/ctu-mrs/mrs_uav_managers/blob/da86b6229468a4dcf2e7d0f4da7c4e18808bdfc2/src/control_manager/control_manager.cpp#L1566C1-L1590C6) :
  * Actuators commands, control group: 250 Hz 
  * *Attitude rate, attitude* (most common): 100 Hz
  * Acceleration + heading rate, acceleration + heading: 40 Hz
  * Velocity + heading rate, velocity + heading, position + heading: 20 Hz
* Consistent dt — The dt between messages should remain stable 
  * 0.01 s between all messages for 100 Hz odometry is good
  * 0.01 s between some messages but sometimes 0.05 s for 100 Hz odometry is bad
* Smooth state variables — No sudden jumps between messages
  * e.g. if a position suddenly jumps 2 meters due to a loop closure, it is bad   
* Body-frame velocities — The linear and angular velocities are in the body (FCU) frame of the UAV  
  * The frame must be specified in the `child_frame_id` of the odometry message (e.g., `uav1/fcu`) 
* Consistend pose and velocity — The pose should correspond with integrated velocities (up to some reasonable drift)

## Running the Passthrough Estimator

The passthrough estimator is available in the MRS system by default; it only needs to be set up and activated.
Follow these steps to run the passthrough estimator in simulation:

1. Specify the passthrough estimator parameters in the `custom_config.yaml` of your tmux session:

```yaml 
mrs_uav_managers:
  estimation_manager:
    # loaded state estimator plugins
    state_estimators: [
      "passthrough",
    ]

    initial_state_estimator: "passthrough" # will be used as the first state estimator
    agl_height_estimator: "" # only slightly filtered height for checking min height (not used in control feedback)

    passthrough:
      max_flight_z: 100.0 # [m] maximum allowed flight Z (in the estimator frame)
      message:
        topic: "hw_api/odometry" # the topic of the odometry that should be passed through
```

2. Specify the constraints and gains for the passthrough estimator in the `custom_config.yaml`:

```yaml
mrs_uav_managers:
  constraint_manager:

    estimator_types: [
      "passthrough"
    ]

    constraints: [
      "slow",
      "medium",
      "fast"
    ]

    # list of allowed gains per odometry mode
    allowed_constraints:
      passthrough: ["slow", "medium", "fast"]

    # those gains will be used automatically when a localization mode switches
    # and the current gains are not in the allowed list (next paragraphs)
    default_constraints:
      passthrough: "slow"

  gain_manager:

    estimator_types: [
      "passthrough"
    ]

    gains: [
      "soft"
    ]

    # list of allowed gains per odometry mode
    allowed_gains:
      passthrough: ["soft"]

    # those gains will be used automatically when a localization mode switches
    # and the current gains are not in the allowed list (next paragraphs)
    default_gains:
      passthrough: "soft"
```

3. Specify the safety area in `world_config.yaml` in the `local_origin` frame:

```yaml 
safety_area:

  enabled: true

  horizontal:

    # the frame of reference in which the points are expressed
    frame_name: "local_origin"

    # polygon
    #
    # x, y [m] for any frame_name except latlon_origin
    # x = latitude, y = longitude [deg]  for frame_name=="latlon_origin"
    points: [
      -50, -50,
      50,  -50,
      50,  50,
      -50, 50,
    ]

  vertical:

    # the frame of reference in which the max&min z is expressed
    frame_name: "local_origin"

    max_z: 30.0
    min_z: 0.5
```

## Example Simulation Session

An example simulation session for running the passthrough estimator is located [here](https://github.com/ctu-mrs/mrs_core_examples/tree/tmux/tmux/passthrough_estimator).
