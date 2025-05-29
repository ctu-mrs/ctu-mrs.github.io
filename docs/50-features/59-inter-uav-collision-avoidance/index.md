---
title: Inter-UAV collision avoidance
pagination_label: Inter-UAV collision avoidance
description: Inter-UAV collision avoidance
---

# Inter-UAV collision avoidance

The mutual collision avoidance of the UAVs is implemented by the [MPC tracker](/docs/features/trackers/).
In short, each UAV shares its **predicted trajectory** with the other UAVs, and checks whether its own predicted trajectory does not collide with the trajectories of the others (with a specified **safety margin**).
If a future collision is detected, a collision avoidance maneuver is executed by the UAV with the lower **priority**:

* Both UAVs slow down.
* The lower-priority UAV (or several UAVs if multiple collisions are predicted) increases its altitude to avoid the collision.

The UAV priority is selected based on the UAV name: the UAV with the higher number has a lower priority.
If a UAV has collision avoidance disabled, then it has the highest priority (all other UAVs with enabled collision avoidance will try to avoid it).

*Note:* This system relies on communication between the UAVs.
Therefore, for collision avoidance to work, **the UAVs must be on the same WiFi**, and sharing of the required messages over the LAN must be configured (see below).
Everything **except the UAV names list** should be set up correctly by default, but it is good practice to double-check before any experiments.

## Configuring the collision avoidance

The parameters of the collision avoidance subsystem are configured in the standard custom config (called simply `custom_config.yaml`) passed to `mrs_uav_core core.launch`.
A tutorial on how to use custom configs is [here](/docs/api/custom_configs/).
The relevant parameters are:
```yaml
mrs_uav_trackers:

  mpc_tracker:

    collision_avoidance:

      enabled:            true  # disabling this will stop this UAV to react to others, but it will still transmit data to others
      enabled_passively:  true  # disabling this will stop this uav to even transmit its data to others (only if enabled: false)
      radius:             5.0   # the collision safety margin - radius used to inflate the predicted trajectories when detecting collisions [m]
      correction:         3.0   # the altitude increase step when avoiding collision [m]
```

*Note:* In case the names or the structure of these paremeters change, you can always find the most up-to-date information using the `get_public_params.py` script (see [here](/docs/api/custom_configs/)).

Furthermore, as mentioned above, the UAVs need to communicate with each other and share their predicted trajectories.
Configuration of the inter-UAV communication has two parts:

1. Specifying the list of UAVs in the group that should avoid collisions.
2. Configuring ROS topics to be shared between the UAVs (the trajectories).

The UAV names list is specified in a special custom config called `network_config.yaml`, that is passed to `mrs_uav_core core.launch` and to `mrs_uav_deployment run_nimbro.py`.

In real-world deployments, the ROS topic sharing between UAVs is implemented using the NimbRo system (see our documentation [here](/docs/features/nimbro-network/)).
In simulation, all topics are available for all UAVs by default, so NimbRo is not required, but the UAV names list still has to be provided in the `network_config.yaml`.
