---
title: Workspace build
pagination_label: Building ROS2 packages in a workspace
description: Building ROS2 packages in a workspace
---
# Workspace build of ROS2 packages

This guide shows how to set up a workspace in ROS2.
If you encounter any issues during the process, see the end of this page for *Troubleshooting* of common issues.


## Installation steps

### 1. Install the Robot Operating System (Jazzy)

  Follow the [ROS2 Installation](https://ctu-mrs.github.io/docs/prerequisites/ros2/installation) guide. 

### 2. Install the MRS UAV Core with necessary dependencies

  Follow the [ROS2 MRS UAV Core Installation](https://ctu-mrs.github.io/docs/installation/ros2-installation) guide. 
  
### 3. Get aliases that make common ROS2 commands usable

By default, calling `colcon build` anywhere creates a workspace there (even in a subdirectory of an workspace).
To prevent that, the aliased `colcon build` crawls back in the directory tree to check if we are in a workspace and if not, will NOT create a workspace automatically.
With the aliases, a workspace can be created only by `colcon init` similarly to `catkin_tools` in ROS1.

```bash
cd ~/git
git clone git@github.com:ctu-mrs/mrs_uav_development.git
cd mrs_uav_development
git checkout ros2
```

Add to `~/.bashrc` (`~/.zshrc`): 
```bash
source $HOME/git/mrs_uav_development/shell_additions/shell_additions.sh
```

### 4. Clone the ROS2 Examples package

```bash
cd ~/git
git clone git@github.com:ctu-mrs/ros2_examples.git
```

### 5. Prepare the workspace

```bash
mkdir -p ~/ws_examples/src
ln -s $HOME/git/ros2_examples $HOME/ws_examples/src/
```

Now we need to set the workspace compilation flags using mixin.

First, install mixin
```bash
sudo apt install python3-colcon-mixin
```

then add default mixin

```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

finally add MRS mixin

```bash
colcon mixin add mrs https://raw.githubusercontent.com/ctu-mrs/mrs_uav_development/refs/heads/ros2/mixin/index.yaml
colcon mixin mrs default
```

Add the following config to `~/ws_examples/colcon_defaults.yaml` to set the number of build threads to 8 and build with the "rel-with-deb-info" mixin profile:
```yaml
build:
  parallel-workers: 8
  mixin:
    - rel-with-deb-info
```

For more information regarding setting workspace flags using mixins see [ROS2 Workspace Profiles](https://ctu-mrs.github.io/docs/prerequisites/ros2/ros1-ros2-patterns/workspace_profiles)

### 6. Build the workspace
```bash
cd ~/ws_examples/
colcon init
colcon build
```

Add to `~/.bashrc` (`~/.zshrc`):
```bash
export ROS_WORKSPACE="$HOME/ws_examples"
```

# Troubleshooting

## Optimization flags 
* *Issue:* Estimation manager is outputting errors with `time since last msg too long`
* *Reason:* Workspace is built without optimization flags.
* *Solution:* See step 5. and make sure the workspace is built with `rel_with_deb_info`

## Tmux config
* *Issue:* After running the tmux session, the terminal looks weird and tmux bindings don't work
* *Reason:* tmux config is missing in /etc/ctu-mrs/tmux.conf
* *Solution:* `sudo apt install mrs-uav-shell-additions`

## MRS UAV testing not building
* *Issue:* The package `mrs_uav_testing` is not building 
* *Reason:* It hasn't been converted to ROS2 yet.
* *Solution:* `touch ~/ws_mrs_uav_core/src/mrs_uav_core/ros_packages/mrs_uav_testing/COLCON_IGNORE`
* 
## Colcon build not working as expected
* *Issue:* Calling colcon build does not create a workspace and start building it.
* *Reason:* We have an alias for colcon, to make its usage less awkward. See step 3 for details.
* *Solution:* First call `colcon init` in the root of the workspace, then `colcon build`.

