---
layout: default
title: mrs_msgs
parent: uav_core
grand_parent: Software
---
# MRS ROS messages

![](fig/thumbnail.jpg)

| Build status | [![Build Status](https://github.com/ctu-mrs/mrs_msgs/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/mrs_msgs/actions) | [![Build Status](https://github.com/ctu-mrs/mrs_msgs/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_msgs/actions) | [![Build Status](https://github.com/ctu-mrs/mrs_msgs/workflows/Docs/badge.svg)](https://github.com/ctu-mrs/mrs_msgs/actions) |
|--------------|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------|

## Documentation

[https://ctu-mrs.github.io/mrs_msgs](https://ctu-mrs.github.io/mrs_msgs).

## General points

* This packages provides definitions of custom messages and services for the whole [MRS system](https://github.com/ctu-mrs/mrs_uav_system).
* No package from within the [system](https://github.com/ctu-mrs/mrs_uav_system) should define custom messages for the following reasons:
  * Dependency graph would be much more interconnected than if all packages depend on this single package.
  * When replaying old rosbags: the only package needed to be compiled in the particular version is this one (which has no other dependencies other than the generic ROS message packages). On the other hand, if all the packages within the system generate the messages, it might be complicated to make them compile in the particular version, given their dependencies might not be satisfied anymore.
