---
title: Autostart Takeoff Routine
pagination_label: Autostart Takeoff Routine
description: Autostart Takeoff Routine
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Autostart takeoff routine

The ROS node for automating takeoff routine for UAVs.
The code originates from the [mrs_uav_autostart](https://github.com/ctu-mrs/mrs_uav_autostart/tree/ros2) package.
The **Autostart** checks the availability of essential parts of the system and validity of received data.
Once all modules and data are available, it waits for the UAV to be armed and switched to offboard mode.
Then, after a __takeoff countdown__, it initiates the takeoff procedure.
During the __takeoff countdown__, takeoff can be aborted by switching the mode back to manual or by disarming the UAV.

The **Autostart** takes the UAV state and the preflight checks from the [DiagnosticsManager](../01-managers/index.md#diagnosticsmanager):

* validity of current position of the UAV (takeoff outside safety area is not allowed),
* limit on current maximum estimated speed (UAV should be static before takeoff),
* the UAV height (if available) needs to suggest that the UAV is on the ground,
* the gyro rates (if available) needs to be near zero,
* availability of data on the [UAV Manager](../01-managers/index.md#uavmanager), [Control Manager](../01-managers/index.md#controlmanager), [Estimation Manager](../01-managers/index.md#estimationmanager) and additional user-specified topics.

## Configuration and use

The autostart node is launched individually (not as part of the core), since it can be relaunched by the user upon request.
The autostart might be even omitted from the session completely if needed.

```bash
ros2 launch mrs_uav_autostart automatic_start.launch.py
```

The autostart is configured through the same [custom config](../../40-api/10-custom_configs.md) as the rest of the MRS UAV System, under the `mrs_uav_autostart:` key.
Pass the same file to the autostart's launch:

```bash
ros2 launch mrs_uav_autostart automatic_start.launch.py custom_config:=./config/custom_config.yaml
```

```yaml
mrs_uav_autostart:

  # countdown between the UAV entering offboard and the takeoff
  takeoff_countdown: 5.0 # [s]

  # false: perform all checks and enable the control output,
  #        but leave calling uav_manager/takeoff to someone else
  trigger_takeoff: true
```

The default values are in the [public config](https://github.com/ctu-mrs/mrs_uav_autostart/blob/ros2/config/public/automatic_start.yaml) and the [private config](https://github.com/ctu-mrs/mrs_uav_autostart/blob/ros2/config/private/automatic_start.yaml) (e.g. `arm_to_output_timeout`); the custom config overrides both.

### Renamed parameters

The following parameters were renamed or moved.
If the custom config contains an old name (with or without the `mrs_uav_autostart:` key), the autostart reports it and shuts down, instead of silently ignoring it.

| old name                 | new name                                               |
|--------------------------|--------------------------------------------------------|
| `safety_timeout`         | `mrs_uav_autostart/takeoff_countdown`                  |
| `handle_takeoff`         | `mrs_uav_autostart/trigger_takeoff`                    |
| `control_output_timeout` | `mrs_uav_autostart/arm_to_output_timeout`              |
| `pre_takeoff_sleep`      | removed, use `mrs_uav_autostart/takeoff_countdown`     |
| `preflight_check`        | `mrs_uav_managers/diagnostics_manager/preflight_check` |

### Custom topics check

The preflight checks are configured in the [DiagnosticsManager](../01-managers/index.md#diagnosticsmanager), under `mrs_uav_managers/diagnostics_manager/preflight_check` in the same custom config.
They also allow the user to specify additional topics that have to be available before initiating takeoff procedure.
If this check is enabled and the latest message received on one of the specified topics is older than __timeout__, the takeoff of the UAV will not be allowed.
Each topic is given as `<topic name>:<message type>`.
A topic name without the leading `/` is prefixed with `/$UAV_NAME/`.
The list replaces the default one, so keep the default topics in it if you want them checked.

```yaml
mrs_uav_managers:
  diagnostics_manager:
    preflight_check:
      topic_check:
        enabled: true
        timeout: 5.0 # [s]
        topics: [
          "estimation_manager/uav_state:mrs_msgs/msg/UavState",
          "control_manager/diagnostics:mrs_msgs/msg/ControlManagerDiagnostics",
          # your topics, e.g. "<topic name>:<message type>",
        ]
```

## Dependencies

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib/tree/ros2)
* [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs/tree/ros2)
