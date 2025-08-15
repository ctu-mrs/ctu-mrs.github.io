---
title: Preparing for a real-world experiment
pagination_label: Preparing for a real-world experiment
description: Preparing for a real-world experiment
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Preparing for a real-world experiment

## Set up your own workspace

You should clone your code into the `~/git` folder, create your own ROS workspace (usually called your_name_workspace) and symlink your code into your workspace.
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
Set up the automatic sourcing of your workspace in the .bashrc file.
Note that only one workspace can be sourced, so comment out any other workspaces that are being sourced.

```bash
# source ~/other_persons_workspace/devel/setup.bash
source ~/john_doe_workspace/devel/setup.bash
```


## You have to prepare your own real-world tmux script.

You should start with our [template tmux script](https://github.com/ctu-mrs/mrs_uav_deployment/tree/master/tmux), copy it into your repository and modify it according to your needs.
This tmux script should run all your nodes, and it should contain panes for calling your services (if necessary).
You should not run anything outside of this tmux script.
