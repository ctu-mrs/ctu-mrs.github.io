---
title: Example sessions
pagination_label: Example sessions with the FlightForge simulator
description: Example sessions with the FlightForge simulator
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Example sessions

The FlightForge simulator comes with a few example sessions that you can use to get started with the simulator. 
The example sessions are located in the `mrs_uav_flightforge_simulation` ROS package.
The invariant part of using the FlightForge simulator is to first start the simulator and then start the ROS nodes that control the drones.

To start the simulator first start the binary from the archive you downloaded in the [installation](/simulations/FlightForge/installation) section.:

```bash
./mrs_flight_forge.sh 
```

Alternatively, you can start the simulator without the GUI:

```bash
./mrs_flight_forge.sh --renderOffScreen
```

## One drone session

Start the one drone tmux session example from the mrs_uav_flightforge_simulation ROS package:
 
```bash
cd /opt/ros/jazzy/share/mrs_uav_flightforge_simulation
./tmux/one_drone/start.sh
```
