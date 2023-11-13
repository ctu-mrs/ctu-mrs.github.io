---
layout: default
title: Preparing for a real-world experiment
parent: The UAV System
nav_order: 7
---

| :warning: **Attention please: This page needs work.**                                                                                             |
| :---                                                                                                                                              |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# Preparing for a real-world experiment:

This guide is intended for newcomers to the MRS group who are planning a real-world experiment, and it is therefore specific to the UAV platforms and setup used by the MRS group.
But all the ideas are also applicable to other people.

### Do not hardcode things.

People often start by hardcoding things like the `UAV_NAME`, or parameters of their programs.
This is very impractical during a real-world experiment.
The `UAV_NAME` is stored in the `$UAV_NAME` environmental variable, and your code should load it and use it.
Configurable parameters of your code should be loaded from a config file, which can be changed without the need to recompile your code.
Use our [mrs_core_examples](https://github.com/ctu-mrs/mrs_core_examples) for inspiration.

### Do not launch new nodes during flight.

Your node should be running even before takeoff, in a "deactivated" state (e.g., not sending "goto" commands).
You should have an activation service, which will be called (manually) after the takeoff is finished.
This way, if your node crashes for some reason, we can see it even before takeoff, which saves time and batteries.
Also, starting a new ros node can be hard on the CPU load, which is not what we want during flight.

### The real world is different than simulations.

You should expect that in the real world, the initial position of the UAV will not be `0,0` and the heading will not be the same every time.
The safety area is constrained by the real-world environment, and therefore it can have an arbitrary shape and orientation.
If you are using a GPS/RTK localization, the X-axis will be aligned with the East direction and Y will be aligned with the North direction.

### Your code has to have human-readable debug/status outputs

Your node should print out its status in a human-readable form.
The output should, for example, contain the current state of your node, information about what data is the node waiting for, what is it publishing, etc.

### Set up your own workspace

You should clone your code into the `~/git` folder, create your own repository (usually called your_name_workspace) and symlink your code into your workspace.
Your workspace has to extend `/opt/ros/noetic`.
Follow these instructions to set up a new workspace:

```bash
mkdir -p ~/john_doe_workspace/src
cd ~/john_doe_workspace
catkin init
catkin config --profile reldeb --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin profile set reldeb
catkin config --extend /opt/ros/noetic
```

Then link/move your packages to the `~/john_doe_workspace/src` and compile the workspace:

```bash
cd ~/john_doe_workspace
catkin build
```

Note that only one workspace can be sourced, so comment out any other workspaces that are being sourced.

```bash
# source ~/other_persons_workspace/devel/setup.bash
source ~/john_doe_workspace/devel/setup.bash
```

### You have to prepare your own real-world tmux script.

You should start with our [template tmux script](https://github.com/ctu-mrs/mrs_uav_deployment/tree/master/tmux), copy it into your repository and modify it according to your needs.
This tmux script should run all your nodes, and it should contain panes for calling your services (if necessary).
You should not run anything outside of this tmux script.
