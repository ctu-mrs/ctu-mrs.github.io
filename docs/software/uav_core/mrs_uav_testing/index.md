---
layout: default
title: mrs_uav_testing
parent: uav_core
grand_parent: Software
---
# MRS UAV Testing [![Build Status](https://travis-ci.com/ctu-mrs/mrs_uav_testing.svg?branch=master)](https://travis-ci.com/ctu-mrs/mrs_uav_testing)

![](fig/mrs_testing.jpg)

## Automated simulation tests of the [uav_core](https://github.com/ctu-mrs/uav_core)

* **gps control test** - series of automated tests of features of the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) and [uav_manager](https://github.com/ctu-mrs/mrs_uav_managers) under GPS localization
* **optic flow control test** - series of automated tests of features of the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) and [uav_manager](https://github.com/ctu-mrs/mrs_uav_managers) under [optic flow](https://github.com/ctu-mrs/mrs_optic_flow) localization
* **bumper test** - random walk in a simulated forest, test the bumper feature of the [control manager](https://github.com/ctu-mrs/mrs_uav_managers) and [MRS bumper](https://github.com/ctu-mrs/mrs_bumper)
* **collision avoidance test** - tests the mutual collision avoidance feature of the [MPC tracker](https://github.com/ctu-mrs/mrs_uav_trackers)
* **safety area test** - tests the safety area feature of the [control manager](https://github.com/ctu-mrs/mrs_uav_managers)
* **circle flier** - control and tracking stress test

## How to

Run tmuxinator sessions in ```tmux/```.
