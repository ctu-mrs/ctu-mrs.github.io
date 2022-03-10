---
layout: default
title: Speed Tracker
parent: The UAV system
nav_order: 8
---

# Flying with the Speed Tracker

The [speed tracker](https://github.com/ctu-mrs/mrs_uav_trackers) is a tracker often used for **swarming**.
It allows direct access to **speed** and/or **acceleration** control reference without much of the post-processing.
The only postprocessing that is applied is a limit to the rate of change of **speed** and **acceleration** in accordance to the current UAV [constraints](https://github.com/ctu-mrs/mrs_uav_managers#constraintmanager).

## Publishing the speed tracker command

The [speed tracker command message](https://ctu-mrs.github.io/mrs_msgs/msg/SpeedTrackerCommand.html) allows to **independently** set the following states:

* 3D acceleration / 3D force
* 3D speed
* height
* heading
* heading rate

Each of the states have to be enabled by a **use_<state>** flag in the message.
Without the flag to be set to **true**, the state will not be controlled by feedback.
The speed, acceleration, force and heading are supposed to be expressed in a [frame of reference](frames_of_reference) identified by the **header/frame_id**.

Topic for publishing the data
```
/<uav_name>/control_manager/speed_tracker/command
```

## Activating the speed tracker

Prior to its activation, messages are supposed to be published on the *command* topic.
The tracker will not activate itself without the data.

Activating the tracker is done via the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#controlmanager) service:
```
/<uav_name>/control_manager/switch_tracker SpeedTracker
```

## in-terminal example

Flying "forward" with constant velocity of 0.5 m/s, in height of 1.5 m:
```bash
rostopic pub /uav1/control_manager/speed_tracker/command mrs_msgs/SpeedTrackerCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'fcu_untilted'
velocity: {x: 0.5, y: 0.0, z: 0.0}
acceleration: {x: 0.0, y: 0.0, z: 0.0}
force: {x: 0.0, y: 0.0, z: 0.0}
height: 1.5
heading: 0.0
heading_rate: 0.0
use_velocity: true 
use_acceleration: false
use_force: false
use_height: true 
use_heading: true 
use_heading_rate: false" -r 10
```

Activating the speed tracker:
```bash
rosservice call /uav1/control_manager/switch_tracker SpeedTracker
```
