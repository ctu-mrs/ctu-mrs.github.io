---
layout: default
title: Hector SLAM
parent: Software
---

| :warning: **Attention please: This page is outdated.**                                                                                           |
| :---                                                                                                                                             |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# Hector SLAM

Hector slam ([github.com/tu-darmstadt-ros-pkg/hector_slam](http://github.com/tu-darmstadt-ros-pkg/hector_slam)) is a 2D LIDAR SLAM.
It was designed to provide 2D localization within a forest, but it performs well within an indoor building environment.

## Getting Hector SLAM

### As an internal member of the CTU lab

The [MRS fork of Hector SLAM](http://github.com/ctu-mrs/hector_slam) can be installed as a part of the [uav_modules](https://mrs.felk.cvut.cz/gitlab/uav/uav_modules) repository from within our [gitlab](http://mrs.felk.cvut.cz/gitlab).

### As a standalone package

Clone the original (vanilla) Hector SLAM
```bash
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```
or our fork
```bash
git clone https://github.com/ctu-mrs/hector_slam.git
```
and place it within your workspace.
Checkout out the **melodic-devel** branch:
```bash
git checkout melodic-devel
```
Compile it *as it is*.

## Launching the SLAM

We provide our custom launch file ([fork](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/hector_slam.launch), [vanilla](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/hector_slam_vanilla.launch)) within the [mrs_uav_general](https://github.com/ctu-mrs/mrs_uav_general) package.
It relies on the *rplidar* sensor, both on a real UAV and in the simulation.

## Examples of use

1. An example simulation tmuxinator session: [one_drone_hector_slam](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts/one_drone_hector_slam)
2. A control test dedicated to Hector SLAM: [test_control_hector](https://github.com/ctu-mrs/mrs_uav_testing/tree/master/tmux/test_control_hector)
3. Basic real-UAV tmux scripts: [just_flying_hector.sh](https://github.com/ctu-mrs/uav_core/blob/master/tmux_scripts/just_flying_hector.sh), [just_flying_hector_optflow](https://github.com/ctu-mrs/uav_core/tree/master/tmux_scripts/just_flying_hector_opflow)
