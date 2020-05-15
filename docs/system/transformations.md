---
layout: default
title: Transformations
parent: The UAV system
---

# Transformations

![](fig/transformations.png)

As a robotic system, the MRS system makes use of many coordinate frames [refererence frames](frames_of_reference.md).
Transformations within the MRS system are maintained by the [tf2 ros package](http://wiki.ros.org/tf2) and [Transformer](https://ctu-mrs.github.io/mrs_lib/classmrs__lib_1_1Transformer.html) from the [mrs_lib package](https://ctu-mrs.github.io/mrs_lib/), which is a tf2 wrapper with several functions simplifying the work with transformations.
The transformations within the MRS system are important not only for tracking the relationship among particular coordinate frames, but they are used also to enable sending commands in various reference frames independently on current control frame. 

## MRS Trackers commands with specified frame_id

As mentioned above, the MRS system provides the opportunity to command a UAV in any coordinate frame with a known transformation to current control frame without need to transform the desired reference explicitly.
The coordinate frame in which the desired reference is provided has to be specified as a `'frame_id'` in a header of the message.
The frame_id of the coordinate frame is formed by part of the coordinate frame name without the name of UAV (e.g., `frame_id = 'fcu_untilted'` for `'<uav_name>/fcu_untilted'`).
Thus the reference trajectory for uav1 given in the 'uav1/fcu_untitlted' coordinate frame can be set by publishing following message: 

```bash
rostopic pub /uav1/control_manager/mpc_tracker/set_trajectory mrs_msgs/TrackerTrajectory
"header:
   seq: 0
   stamp: {secs: 0, nsecs: 0}
   frame_id: 'fcu_untilted'
 use_yaw: false
 fly_now: true
 loop: false
 points:
 - {x: 10.0, y: 0.0, z: 0.0, yaw: 0.0}
 - {x: 11.0, y: 0.0, z: 0.0, yaw: 0.0}
 - {x: 12.0, y: 0.0, z: 0.0, yaw: 0.0}"
```

Without specifying the *frame_id*, the currently frame will be used.
If an invalid *frame_id* is given, the reference is not used. 
