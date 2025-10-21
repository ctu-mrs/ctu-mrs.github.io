---
title: ROS2 native installation
pagination_label: Installing the MRS ROS2 natively
description: Installing the MRS ROS2 natively
---

# Native installation of the ROS2 MRS UAV System

This guide shows how to install the ROS2 MRS UAV System natively.

## Installation steps

### 1. Install the Robot Operating System (Jazzy)

Follow the [Installation](../10-prerequisites/25-ros2/10-installation.md) guide. 

### 2. Install the MRS UAV Core

Add the Personal Package Archive (PPA) for unstable ROS2 MRS deb packages:
```bash
curl https://ctu-mrs.github.io/ppa2-unstable/add_ppa.sh | bash
sudo apt update
```

Install the MRS UAV System deb packages and dependencies:
```bash
sudo apt install ros-jazzy-mrs-uav-core
```

### 2. Set Zenoh to be the used RMW implementation

Add to `~/.bashrc` (`~/.zshrc`):
```
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
```

Source `~/.bashrc` (`~/.zshrc`):
```bash
source ~/.bashrc
```

### 3. Run the multirotor simulation session to verify the installation
```bash
cd /opt/ros/jazzy/share/mrs_multirotor_simulator/tmux/mrs_one_drone
./start.sh
```

