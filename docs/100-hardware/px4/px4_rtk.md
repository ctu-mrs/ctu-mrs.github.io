---
title: PX4 RTK Knowledge Base
---

### PX4 RTK Parameters for flight
---

Following are the parameters that have to be changed to fly on the F9P Helical RTK system.

`EKF2_GPS_P_NOISE` = 0.2
`EKF2_GPS_V_NOISE` = 0.2
`EKF2_HGT_REF` = 1 to fly on GPS altitude
`GPS_UBX_DYNMODEL` = 8 for flying at the \<4g civilian limit of accelerations

### PX4 HW API Changes
---

It was observed that on `v1.14.0`, the mavros topic `/mavros/odometry/in` used by the PX4 `api.launch` for new PX4 boards is providing velocity estimates in the wrong frame and angular velocities are noisy. So, it is recommended to move the `api.launch` topic remap to: 

```xml
<remap if="$(eval arg('OLD_PX4_FW') == 0)" from="~mavros_local_position_in" to="mavros/odometry/in" />
<remap if="$(eval arg('OLD_PX4_FW') == 1)" from="~mavros_local_position_in" to="mavros/local_position/odom" />
```

This allows you to fly on PX4's own odometry which has been verified by the agile team to be accurate to centimeters. `gps_baro` estimator does not fuse velocities and therefore, does not run into the issue of inconsistent velocities.

