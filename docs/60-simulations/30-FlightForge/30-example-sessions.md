---
title: Example sessions
pagination_label: Example sessions with the FlightForge simulator
description: Example sessions with the FlightForge simulator
---

# Example sessions

The FlightForge simulator comes with a few example sessions that you can use to get started with the simulator. 
The example sessions are located in the `mrs_uav_unreal_simulation` ROS package.
The invarient part of using the FlightForge simulator is to first start the simulator and then start the ROS nodes that control the drones.

To start the simulator first start the binary from the archive you downloaded in the [installation](10-installation.md) section.:

```bash
./flightforge.sh 
```

Alternatively, you can start the simulator without the GUI:

```bash
./flightforge.sh --renderOffScreen
```



## One drone session

Start the one drone tmux session example from the mrs_uav_unreal_simulation ROS package:
 
```bash
roscd mrs_uav_unreal_simulation
./tmux/one_drone/start.sh
```


## Two dones session

Start the two drones tmux session example from the mrs_uav_unreal_simulation ROS package:

```bash
roscd mrs_uav_unreal_simulation
./tmux/two_drones/start.sh
```
