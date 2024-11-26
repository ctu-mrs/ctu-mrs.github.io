---
title: Standalone in ROS
---

# Prerequsities

1. The [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed.
2. The [MRS UAV Core](http://github.com/ctu-mrs/mrs_uav_core) installed.

# Starting the simulation

Running the simulation consists of several steps, which are **automated** using a **tmuxinator** script.
Examples of tmuxinator files can be found in the **tmux** folder within the [mrs_multirotor_simulator](https://github.com/ctu-mrs/mrs_multirotor_simulator) package.
With the system installed, you can find them (and then copy them elsewhere) by running
```bash
roscd mrs_multirotor_simulator/tmux
```
Each folder contains a different simulation scenario.

The simlation scenario will be started by calling the `start.sh` script within its folder.
This will call the `tmuxinator` utility that uses the prescription in the `session.yml` file to spawn a `tmux` session.
```bash
roscd mrs_multirotor_simulator/tmux/standalone
./start.sh
```

You should be presented with the "Rviz" window upon starting the session.
![](fig/mrs_standalone_windows.png)

A similar session was also prepared to showcase the simulator of large number (400) of UAVs:

```bash
roscd mrs_multirotor_simulator/tmux/standalone_400_uavs
./start.sh
```

![](fig/mrs_standalone_400_windows.png)

# Stopping the simulation

You have two options:

a) Run the `./kill.sh` script.
b) Hit the _killing shortcut_: `ctrl+a k` (`ctrl+a` and then `k`). A menu will appear in which you confirm the selection.

# Configuring the simulator

The simulator is configure through a _custom config_ file [here](https://github.com/ctu-mrs/mrs_multirotor_simulator/blob/master/tmux/mrs_one_drone/config/simulator.yaml) which contains the differences form the [default simulator config file](https://github.com/ctu-mrs/mrs_multirotor_simulator/blob/master/config/multirotor_simulator.yaml).
You can configure how many drones should be spawned, of which type and where.
The particular drone models are configure [here](https://github.com/ctu-mrs/mrs_multirotor_simulator/tree/master/config/uavs).
