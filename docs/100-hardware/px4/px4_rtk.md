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

It was observed that on `v1.14.0`, the mavros topic `/mavros/odometry/in` used by the PX4 `api.launch` for new PX4 boards is providing velocity estimates in the wrong frame and angular velocities are noisy. So, it is recommended to move the `api.launch` topic remap to the `/mavros/local_position/odom` topic.

```xml
<remap if="$(eval arg('OLD_PX4_FW') == 0)" from="~mavros_local_position_in" to="mavros/local_position/odom" />
<remap if="$(eval arg('OLD_PX4_FW') == 1)" from="~mavros_local_position_in" to="mavros/local_position/odom" />
```

This allows you to fly on PX4's own odometry which has been verified by the agile team to be accurate to centimeters. `gps_baro` estimator does not fuse velocities and therefore, does not run into the issue of inconsistent velocities.

#### Running a Single EKF Instance
> [!WARNING] 
> **This has not been tested yet!**

You may notice in your data that the odometry from PX4 sometimes suddenly jumps for no apparent reason. This could be caused by the EKF estimator switching inside PX4, as it can run multiple EKF estimators and select the one that is the healthiest.
To run a single EKF in PX4, you should set:
  `EKF2_MULTI_IMU`=0  
  `EKF2_MULTI_MAG`=0  
  `SENS_IMU_MODE`=1  
  `SENS_MAG_MODE`=1  
This provides protection against a limited number of sensor faults, such as loss of data, but does not protect against the sensor providing inaccurate data that exceeds the ability of the EKF and control loops to compensate.
