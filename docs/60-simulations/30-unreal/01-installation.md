---
title: Installation
pagination_label: Installing the Unreal Engine simulator
description: How to install the Unreal Engine simulator
---

# Prerequisites

1. The [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed.
2. The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) installed.
3. The [MRS UAV Unreal Simulation](https://github.com/ctu-mrs/mrs_uav_unreal_simulation) installed.
4. Download the latest binary of the Unreal Engine simulator from the [Releses page](https://nasmrs.felk.cvut.cz/index.php/s/MnGARsSwnpeVy5z).

# Starting the simulation

Running the simulation consists of several steps, which are mostly **automated** using a **tmuxinator** script.
The script does not start the Unreal based simulator **FlightForge** itself, so you need to start the simualtion manually from your download directory by running
```bash 
./FlightForge.sh
```
Examples of tmuxinator files can be found in the **tmux** folder within the [mrs_uav_unreal_simulation](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation) package.
With the system installed, you can find them (and then copy them elsewhere) by running
```bash
roscd mrs_uav_unreal_simulation/tmux
```
Each folder contains a different simulation scenario.

The simlation scenario will be started by calling the `start.sh` script within its folder.
This will call the `tmuxinator` utility that uses the prescription in the `session.yml` file to spawn a `tmux` session.
```bash
roscd mrs_uav_unreal_simulation/tmux/one_drone
./start.sh
```

You should be presented with the windows of the "FlightForge simulator" and the "Rviz" after starting the session.

# Stopping the simulation

To stop the **FlightForge** simulator you just kill it in the terminal where you started it by pressing `ctrl+c`.

To stop the rest of the simulation you have two options:

a) Run the `./kill.sh` script.
b) Hit the _killing shortcut_: `ctrl+a k` (`ctrl+a` and then `k`). A menu will appear in which you confirm the selection.

# Configuring the UAV

To confugure the UAV, you need to will have to edit the `simulator.yaml` file in the `config` folder of the tmux session folder.
