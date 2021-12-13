---
layout: default
title: Preparing for a real-world experiment
parent: The UAV system
nav_order: 5
---

# Requirements:
**Do not hardcode things.
A lot of people start by hardcoding things like the UAV_NAME, or parameters of their programs. This is very impractical during a real-world experiment. The UAV_NAME is stored in the $UAV_NAME environmental variable, and you code should load it and use it. Configurable parameters of your code should be loaded from a config file, which can be changed without the need to recompile your code. Use this [example ros node](https://github.com/ctu-mrs/example_ros_uav) as inspiration.

**Do not launch new nodes during flight.
Your node should be running even before takeoff, in a "deactivated" state (e.g. not sending "goto" commands). You should have an activation service, which will be called (manually) after the takeoff is finished. This way, if your node crashes for some reason, we can see it even before takeoff, which saves time and batteries. Also, starting a new ros node can be hard on the cpu load, which is not what we want during flight.

**The real world is different than simulations.
You should expect that in the real world, the initial position of the uav will not be 0,0 and the heading will not be the same every time. The safety area is constrained by the real-world environment, and therefore it can have an arbitrary shape and orientation. If you are using a GPS/RTK localization, the X axis will be aligned with East direction and Y will be aligned with North direction. 

**Your code has to have human-readable debug/status outputs
Your node should print out its status in a human-readable form. The output should, for example, contain the current status of your node, information like what data is the node waiting for, what is it publishing, etc.

**Set up your own workspace
By default, the uav has two workspaces. The mrs_workspace contains uav_core and the modules_workspace has uav_modules. The modules_workspace is extending the mrs_workspace. You should clone your code into the ~git folder, create your own repository (usually called your_name_workspace) and symlink your code into the workspace. Your workspace has to extend the modules_workspace.

```bash
cd ~/
mkdir your_name_workspace
cd your_name_workspace
catkin init
catkin config --extend ~/modules_workspace/devel
mkdir src
cd src
ln -s ~/git/your_code
cd ..
catkin build
```

You also have to add your workspace into the ROS_WORKSPACES variable inside .bashrc, and add sourcing of your_name_workspace/devel/setup.bash. Note that only one workspace can be sourced, so comment out any other workspaces that are being sourced (modules_workspace by default).

**You have to prepare your own real-world tmux script.
You should start with our [template tmux script] (https://github.com/ctu-mrs/uav_core/tree/master/tmux_scripts/swarming_template), copy it into your repository and modify it according to your needs. This tmux script should run all your nodes and it should contain panes for calling your services (if necessary). You should not run anything outside of this tmux script.
