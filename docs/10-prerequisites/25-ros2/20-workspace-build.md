---
title: Workspace build
pagination_label: Building ROS2 packages in a workspace
description: Building ROS2 packages in a workspace
---

# Workspace build of ROS2 packages

This guide shows how to set up a workspace in ROS2. If you encounter any issues during the process, see the end of this page for *Troubleshooting* of common issues.

## Installation steps

### 1. Install the Robot Operating System (Jazzy)

Follow the [ROS2 Installation](./10-installation.md) guide.

### 2. Install the MRS UAV Core with necessary dependencies

Follow the [ROS2 MRS UAV Core Installation](../../20-installation/index.md) guide.

### 3. Get aliases that make common ROS2 commands usable

By default, calling `colcon build` anywhere creates a workspace there (even in a subdirectory of an workspace). To prevent that, the aliased `colcon build` crawls back in the directory tree to check if we are in a workspace and if not, will NOT create a workspace automatically. With the aliases, a workspace can be created only by `colcon init` similarly to `catkin_tools` in ROS1.

```bash
cd ~/git
git clone git@github.com:ctu-mrs/mrs_uav_development.git
cd mrs_uav_development
git checkout ros2
```

Add to `~/.bashrc` (`~/.zshrc`):

```bash
# ROS DEVELOPMENT
# * source this after exporting $ROS_WORKSPACE="<path to your workspace>"
# * workspace is automatically sourced and the soucing is cached
# * to force-source a workspace after adding new packages, call `presource_ros`
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

First, install mixin and the extension which allows defining overrides

```bash
sudo apt install python3-colcon-mixin python3-colcon-override-check
```

then add the MRS mixin.

```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin add mrs https://raw.githubusercontent.com/ctu-mrs/colcon_mixin/refs/heads/master/index.yaml
colcon mixin update
```

Add the following config to `~/ws_examples/colcon_defaults.yaml` to set the number of build workers to 4 and build with the "rel-with-deb-info" and "compile-commands" mixin profiles:

```yaml
build:
  parallel-workers: 4
  mixin:
    - rel-with-deb-info
    - compile-commands
  allow-overriding: []
```

If you want to build MRS integration and unit tests, add the _mrs-testing_ mixin:

```yaml
build:
  mixin:
    - rel-with-deb-info
    - compile-commands
    - mrs-testing
  allow-overriding: []
```

For more information regarding setting workspace flags using mixins see [ROS2 Workspace Profiles](50-ros1-ros2-patterns/70-workspace_profiles.md)

### 6. Build the workspace

```bash
cd ~/ws_examples/
colcon init
colcon build
```

Add this to `~/.bashrc` (`~/.zshrc`) before the sourcing of `source $HOME/git/mrs_uav_development/shell_additions/shell_additions.sh`:

```bash
## workspace to be sourced
export ROS_WORKSPACE="$HOME/ws_examples"
```

Do not put any manual sourcing of the workspace or `/opt/ros/jazzy/` into your `~/.bashrc` (`~/.zshrc`)!

Sourcing `mrs_uav_development` sources the `ROS_WORKSPACE` specified in `~/.bashrc` (`~/.zshrc`) automatically (or it sources `/opt/ros/jazzy` if no `ROS_WORKSPACE` is specified.

### 7. Colorful output for colcon commands

If you're accustomed to the colorful output of catkin_tools from ROS 1, you can achieve a similar look and feel with colcon in ROS 2. This makes it easier to read build logs and quickly identify important messages or errors.

To get started, install the colcon-ansi-colors-example extension using pip:

```bash
pip install git+https://github.com/cottsay/colcon-ansi-colors-example --break-system-packages
```

Note: The `--break-system-packages` flag is often required to install pip packages globally on systems where the package manager is strict about system files.

Once the extension is installed, you can use the new output style with the following command:

```bash
colcon --output-style catkin_tools build
```

If you like the colorful output and want to use it by default for all colcon commands, add the following line to your shell's configuration file (e.g., `~/.bashrc` for Bash or `~/.zshrc` for Zsh):

```bash
export COLCON_DEFAULT_OUTPUT_STYLE=catkin_tools
```

After adding the line, be sure to open a new terminal or run `source ~/.bashrc` (or the appropriate file) for the change to take effect.

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

## Colcon build not creating a workspace

* *Issue:* Calling colcon build does not create a workspace and start building it.
* *Reason:* We have an alias for colcon, to make its usage less awkward. See step 3 for details.
* *Solution:* First call `colcon init` in the root of the workspace, then `colcon build`.

## Colcon build gives a warning about unknown keys

* *Issue:* WARNING:colcon.colcon_defaults.argument_parser.defaults:Skipping unknown keys for 'build': allow-overriding.
* *Reason:* The key comes from the mixin or override_check extension (see `colcon extensions`).
* *Solution:* `sudo apt install python3-colcon-mixin python3-colcon-override-check`.

## Colcon build fails to find ament_package

* *Issue:* Calling colcon build fails with error: `ModuleNotFoundError: No module named 'ament_package'`
* *Reason:* `ROS_WORKSPACE` is sourced before building the workspace.
* *Solution:* Remove `ROS_WORKSPACE` from `~/.bashrc` (`~/.zshrc`), open new terminal, build the workspace, return `ROS_WORKSPACE` to `~/.bashrc` (`~/.zshrc`).

## Any ros2 command fails

* *Issue:* Calling any ros2 commands fails with `importlib.metadata.PackageNotFoundError: No package metadata was found for ros2cli`
* *Reason:* `/tmp/ros_presource_output.sh` exists and needs to be deleted.
* *Solution:* Remove `rm /tmp/ros_presource_output.sh`
