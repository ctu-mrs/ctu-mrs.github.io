---
title: CoppeliaSim
pagination_label: CoppeliaSim simulator
description: CoppeliaSim simulator
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Prerequsities

1. The [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) installed.
2. The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) installed.
3. The [MRS UAV Coppelia Simulation](https://github.com/ctu-mrs/mrs_uav_coppelia_simulation) installed.

# Starting the simulation

Running the simulation consists of several steps, which are **automated** using a **tmuxinator** script.
Examples of tmuxinator files can be found in the **tmux** folder within the [mrs_uav_coppelia_simulation](https://github.com/ctu-mrs/mrs_uav_coppelia_simulation) package.
With the system installed, you can find it (and then copy it elsewhere) by running
```bash
roscd mrs_uav_coppelia_simulation/tmux
```

The simlation scenario will be started by calling the `start.sh` script within its folder.
This will call the `tmuxinator` utility that uses the prescription in the `session.yml` file to spawn a `tmux` session.
```bash
roscd mrs_uav_coppelia_simulation/tmux
./start.sh
```

When the CoppeliaSim window appears, click on the "**play**" button to start/resume the simulation.
Then, the MRS UAV System will start and RViz window will appear.
![](fig/coppelia_windows.png)

# Stopping the simulation

You have two options:

a) Run the `./kill.sh` script.
b) Hit the _killing shortcut_: `ctrl+a k` (`ctrl+a` and then `k`). A menu will appear in which you confirm the selection.

# How does it work?

For more details, see the [mrs_uav_coppelia_simulation](https://github.com/ctu-mrs/mrs_uav_coppelia_simulation) repository.
