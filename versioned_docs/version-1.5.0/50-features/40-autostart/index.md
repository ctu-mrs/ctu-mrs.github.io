---
title: Autostart Takeoff Routine
pagination_label: Autostart Takeoff Routine
description: Autostart Takeoff Routine
---

# Autostart takeoff routine

The ROS node for automating takeoff routine for UAVs.
The code originates from the [mrs_uav_autostart](https://github.com/ctu-mrs/mrs_uav_autostart) package.
The **Autostart** checks the availability of essential parts of the system and validity of received data.
Once all modules and data are available, it waits for the UAV to be armed and switched to offboard mode.
Then, after a __safety timeout__, it initiates the takeoff procedure.
During the __safety timeout__ period, takeoff can be aborted by switching the mode back to manual or by disarming the UAV.

The **Autostart** performs following checks:

* availability of [UAV Manager](https://ctu-mrs.github.io/docs/features/managers),
* availability of [Control Manager](https://ctu-mrs.github.io/docs/features/managers),
* availability of [Estimation Manager](https://ctu-mrs.github.io/docs/features/managers),
* connection to [HW Api](https://ctu-mrs.github.io/docs/plugin-interface/hardware-api/),
* validity of current position of the UAV (takeoff outside safety area is not allowed),
* limit on current maximum estimated speed (UAV should be static before takeoff),
* the UAV height (if available) needs to suggest that the UAV is on the ground,
* the gyro rates (if available) needs to be near zero,
* availability of data on additional user-specified topics.

## Configuration and use

The autostart node is launched individually (not as part of the core), since it can be relaunched by the user upon request.
The autostart might be even omitted from the session completely if needed.

```bash
roslaunch mrs_uav_autostart automatic_start.launch
```

A custom config file can be passed to autostart's launch if needed:
```bash
roslaunch mrs_uav_autostart automatic_start.launch custom_config:=./config/automatic_start.yaml
```
### Custom topics check

The **Autostart** node allows the user to specify additional topics that have to be available before initiating takeoff procedure.
If this check is enabled and the latest message received on one of the specified topics is older than __timeout__, the takeoff of the UAV will not be allowed.

```yaml
preflight_check:
  topic_check:
    enabled: true
    timeout: 5.0 # [s]
    topics: [
      "estimation_manager/uav_state",
      "/mission_controller/diagnostics"
    ]
```

## Dependencies

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib)
* [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs)
