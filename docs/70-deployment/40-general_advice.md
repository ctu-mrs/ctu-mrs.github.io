---
title: General advice
pagination_label: General advice for deployment
description: General advice for deployment
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# General advice for deployment

## Do not hardcode things.

People often start by hardcoding things like the `UAV_NAME`, or parameters of their programs.
This is very impractical during a real-world experiment.
The `UAV_NAME` is stored in the `$UAV_NAME` environmental variable, and your code should load it and use it.
Configurable parameters of your code should be loaded from a config file, which can be changed without the need to recompile your code.
Use our [mrs_core_examples](https://github.com/ctu-mrs/mrs_core_examples) for inspiration.

## Do not launch new nodes during flight.

Your node should be running even before takeoff, in a "deactivated" state (e.g., not sending "goto" commands).
You should have an activation service, which will be called (manually) after the takeoff is finished.
This way, if your node crashes for some reason, we can see it even before takeoff, which saves time and batteries.
Also, starting a new ROS node can produce spikes in the CPU load, which is not wanted during flight.

## The real world is different than simulations.

You should expect that in the real world, the initial position of the UAV will not be `0,0` and the heading will not be the same every time.
The safety area is constrained by the real-world environment, and therefore it can have an arbitrary shape and orientation.
If you are using a GPS/RTK localization, the X-axis will be aligned with the East direction and Y will be aligned with the North direction.

## Your code has to have human-readable debug/status outputs

Your node should print out its status in a human-readable form.
The output should, for example, contain the current state of your node, information about what data is the node waiting for, what is it publishing, etc.
