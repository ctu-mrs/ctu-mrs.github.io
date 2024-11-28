---
title: Obstacle bumper
pagination_label: Obstacle bumper
description: Obstacle bumper
---

# Obstacle bumper

![](fig/bumper.jpg)

## Data aggregation

The [MRS bumper](https://github.com/ctu-mrs/mrs_bumper) aggregates data from 1-D, 2-D lidars, and depth-camera images and creates a sector-based representation of the surroundings of a robot.
The advertised *obstacle sectors* message can be visualized in Rviz using our [Rviz plugins](/docs/features/rviz_plugins/) and can be used by other ROS nodes in real-time.
The data is used in the [control manager](/docs/features/managers/), for its obstacle avoidance feature.

The data aggregation does not run automatically, but it needs to be launched on-demand byu the user:
```bash
roslaunch mrs_bumper bumper.launch
```

A custom config can be passed to the launch file as:
```bash
roslaunch mrs_bumper bumper.launch custom_config:=<path to your file>
```

## Configuration and behavior

TODO
