---
title: ROS2 workspace installation
pagination_label: Building the MRS ROS2 in workspace
description: Building the MRS ROS2 in workspace
---

# Workspace build of the ROS2 MRS UAV System

This guide is meant mainly for developers of the ROS2 MRS UAV System.
Users of the system that do not need to modify the core of the system can stick to the deb installation

## Installation steps

### 1. Install the Robot Operating System (Jazzy)

  Follow the [Installation](https://ctu-mrs.github.io/docs/prerequisities/ros2/installation) guide. 

### 2. Install necessary dependencies

Add the Personal Package Archive (PPA) for unstable ROS2 MRS deb packages:
```bash
curl https://ctu-mrs.github.io/ppa2-unstable/add_ppa.sh | bash
sudo apt update
```

Install the dependencies:
```bash
sudo apt install mrs-uav-shell-additions ros-jazzy-nlopt ros-jazzy-rmw-zenoh-cpp
```
  * `mrs-uav-shell-additions` - installs tmux config file to `/etc/ctu-mrs/tmux.conf` (necessary for running simulation tmux sessions)
  * `ros-jazzy-rmw-zenoh-cpp` - the ROS MiddleWare (RMW) implementation of Zenoh communication protocol
  * `ros-jazzy-nlopt` - necessary for `mrs_uav_trajectory_generation` package

Add to `~/.bashrc` (`~/.zshrc`):
```
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
```

### 3. Get aliases that make common ROS2 commands usable

```bash
cd ~/git
git clone git@github.com:ctu-mrs/mrs_uav_development.git
cd mrs_uav_development
git checkout ros2
```

Add to `~/.bashrc` (`~/.zshrc`): 
```bash
source $GIT_PATH/mrs_uav_development/shell_additions/shell_additions.sh
```

### 4. Clone the MRS UAV Core repository

```bash
cd ~/git
git clone git@github.com:ctu-mrs/mrs_uav_core.git
cd mrs_uav_core
git checkout ros2
gitman install
```

### 5. Prepare the workspace

```bash
mkdir -p ~/ws_mrs_uav_core/src
ln -s $GIT_PATH/mrs_uav_core $HOME/ws_mrs_uav_core/src/
```

Set the compilation flags to `rel-with-deb-info` using mixin according to:
https://ctu-mrs.github.io/docs/prerequisities/ros2/ros1-ros2-patterns/workspace_profiles

First, install mixin:
```bash
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Add the following config to `~/ws_mrs_uav_core/colcon_default.yaml`:
```yaml
build:
  parallel-workers: 8
  mixin:
    - rel-with-deb-info
```

Add to `~/.bashrc` (`~/.zshrc`):
```bash
source /opt/ros/jazzy/setup.bash
```

Source `~/.bashrc` (`~/.zshrc`):
```bash
source ~/.bashrc
```

### 6. Build the workspace
```bash
cd ~/ws_mrs_uav_core/
colcon build
```

Add to `~/.bashrc` (`~/.zshrc`):
```bash
source $HOME/ws_mrs_uav_core/install/setup.bash
```

### 7. Run the multirotor simulation session to verify the installation
```bash
cd ~/ws_mrs_uav_core/src/mrs_uav_core/ros_packages/mrs_multirotor_simulator/tmux/mrs_one_drone/
./start.sh
```

# Troubleshooting

## Optimization flags 
* Issue: Estimation manager is outputting errors with `time since last msg too long`
* Reason: Workspace is built without optimization flags.
* Solution: See step 5. and make sure the workspace is built with `rel_with_deb_info`

## Tmux config
* Issue: After running the tmux session, the terminal looks weird and tmux bindings don't work
* Reason: tmux config is missing in /etc/ctu-mrs/tmux.conf
* Solution: `sudo apt install mrs-uav-shell-additions`

## MRS UAV testing not building
* Issue: The package `mrs_uav_testing` is not building 
* Reason: It hasn't been converted to ROS2 yet.
* Solution: `touch ~/ws_mrs_uav_core/src/mrs_uav_core/ros_packages/mrs_uav_testing/COLCON_IGNORE`

