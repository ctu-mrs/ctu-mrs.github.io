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

Follow the generic [tutorial](https://ctu-mrs.github.io/docs/prerequisites/ros2/workspace-build) on how to manage ROS2 colcon workspace.

## Start tailoring your own real-world tmux script

You should start with our [template tmux script](https://github.com/ctu-mrs/mrs_uav_deployment/tree/ros2/tmux), copy it into your repository and modify it according to your needs.
This tmux script should run all your nodes, and it should contain panes for calling your services (if necessary).
You should not run anything outside of this tmux script.
