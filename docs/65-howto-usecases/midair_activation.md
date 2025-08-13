---
title: Midair activation
pagination_label: Midair activation of the MRS system
description: Midair activation of the MRS system
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Midair activation

For the cases of flight where you need the aircraft to either activate in the air or be flight-ready on the ground without having to perform a take-off procedure, you can use the midair activation feature. To use this, follow this procedure:

1. In your custom configuration file, set the controller and tracker you desire to be used when midair activation is called:
```yaml
    midair_activation:
      after_activation:
        controller: "MpcController"
        tracker: "NullTracker"
```
2. Arm your UAV.
3. Call midair activation using the service call `rosservice call /<vehicle_name>/uav_manager/midair_activation`. This service will request the underlying `hw_api` to switch to offboard and when the status is confirmed, it will switch to the controller and tracker specified in the configuration file.
4. The UAV will now be engaged with the controller and tracker specified in the configuration file.
