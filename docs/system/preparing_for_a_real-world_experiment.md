---
layout: default
title: Preparing for a real-world experiment
parent: The UAV system
nav_order: 7
---

# Preparing for a real-world experiment:

This guide is intended for newcomers to the MRS group who are planning a real-world experiment, and it is therefore specific to the UAV platforms and setup used by the MRS group. But all the ideas are also applicable to other people.

### Do not hardcode things.
People often start by hardcoding things like the UAV_NAME, or parameters of their programs. This is very impractical during a real-world experiment. The UAV_NAME is stored in the $UAV_NAME environmental variable, and your code should load it and use it. Configurable parameters of your code should be loaded from a config file, which can be changed without the need to recompile your code. Use our [example ros node](https://github.com/ctu-mrs/example_ros_uav) as inspiration.

### Do not launch new nodes during flight.
Your node should be running even before takeoff, in a "deactivated" state (e.g. not sending "goto" commands). You should have an activation service, which will be called (manually) after the takeoff is finished. This way, if your node crashes for some reason, we can see it even before takeoff, which saves time and batteries. Also, starting a new ros node can be hard on the CPU load, which is not what we want during flight.

### The real world is different than simulations.
You should expect that in the real world, the initial position of the UAV will not be 0,0 and the heading will not be the same every time. The safety area is constrained by the real-world environment, and therefore it can have an arbitrary shape and orientation. If you are using a GPS/RTK localization, the X-axis will be aligned with the East direction and Y will be aligned with the North direction. 

### Your code has to have human-readable debug/status outputs
Your node should print out its status in a human-readable form. The output should, for example, contain the current state of your node, information about what data is the node waiting for, what is it publishing, etc.

### Set up your own workspace
By default, the UAV has two workspaces. The mrs_workspace contains uav_core and the modules_workspace contains uav_modules. The modules_workspace is extending the mrs_workspace. You should clone your code into the ~/git folder, create your own repository (usually called your_name_workspace) and symlink your code into your workspace. Your workspace has to extend the modules_workspace. Follow these instructions to set up a new workspace:

```bash
cd ~/
mkdir john_doe_workspace
cd john_doe_workspace
catkin init
catkin config --extend ~/modules_workspace/devel
mkdir src
cd src
ln -s ~/git/your_code
cd ..
catkin build
```

You also have to add your workspace into the ROS_WORKSPACES variable inside .bashrc and add sourcing of your_name_workspace/devel/setup.bash. Note that only one workspace can be sourced, so comment out any other workspaces that are being sourced (modules_workspace by default).


```bash
# source ~/mrs_workspace/devel/setup.zsh
# source ~/modules_workspace/devel/setup.zsh
source ~/john_doe_workspace/devel/setup.zsh

export ROS_WORKSPACES="~/modules_workspace ~/mrs_workspace ~/john_doe_workspace"
```

### You have to prepare your own real-world tmux script.
You should start with our [template tmux script](https://github.com/ctu-mrs/uav_core/tree/master/tmux_scripts/swarming_template), copy it into your repository and modify it according to your needs. This tmux script should run all your nodes, and it should contain panes for calling your services (if necessary). You should not run anything outside of this tmux script.
