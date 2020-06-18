---
layout: default
title: How to update
parent: Introduction
nav_order: 98
---

# How to update the mrs_uav_system

This manual assumes you have already installed the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) and you want to update package(s) of the MRS system.
Before reading this tutorial, we recommend to read the [**Gitman manual**](https://ctu-mrs.github.io/docs/software/gitman.html).

## 1. Update *uav_core* and its submodule(s)

Navigate to MRS [uav_core](https://github.com/ctu-mrs/uav_core).
If you followed our installation process, *uav_core* is located in:
```bash
cd ~/git/uav_core
```

Pull the stable version of each submodule on your local machine:
```bash
git pull
```
Remark that the `git pull` command is an alias to [`gitman install`](https://ctu-mrs.github.io/docs/software/gitman.html) --- the alias is defined in [uav_core shell additions](https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/shell_additions/shell_additions.sh) and works only in root of a Gitman-maintained git repository.

To checkout the newest commit instead of the stable version, you may follow `git pull` (but see the [Gitman manual](https://ctu-mrs.github.io/docs/software/gitman.html) first) with:
```bash
gitman update
```

## 2. Update *simulation* and its submodule(s)
Similarly to updating the *uav_core*, checkout stable version of *simulation*:
```bash
cd ~/git/simulation
git pull
```

## 3. Recompile `~/mrs_workspace`
```bash
cd ~/mrs_workspace
catkin build
```
Follow the [`How to compile`](https://ctu-mrs.github.io/docs/introduction/how_to_compile.html) manual for more information.

## 4. Optional: Update [linux-setup](https://github.com/klaxalk/linux-setup)
```bash
cd ~/git/linux-setup
git pull
```
In case the *bash/zshell/vim* environment breaks after running `git pull`, rerun [linux-setup installation](https://github.com/klaxalk/linux-setup/blob/master/install.sh) to setup all the dependencies.

## FAQ

### How to update only selected submodules

If you want to update just few packages, follow `git pull` in a Gitman-maintained git repository (*uav_core*/*simulation*) with:
```bash
gitman install <package_name_1> ... <package_name_X>
```
or
```bash
gitman update <package_name_1> ... <package_name_X>
```

Another way is to checkout specific packages manually, e.g., to checkout `master` branch of `mrs_uav_odometry`:
```bash
roscd mrs_uav_odometry
git checkout master
git pull
```

Do not forget to [**compile**](https://ctu-mrs.github.io/docs/introduction/how_to_compile.html) the packages or the entire workspace after. 

### `git pull` failed: uncommitted changes

You have uncommitted local changes in the current git repository or in one of its submodules.
Throw away, stash, or commit your local changes and run `git pull` again.
To throw away uncommitted changes within a package (beware, you will loose all your changes!):
```bash
roscd <package_name>
git reset --hard
```

### `git pull`: submodules were not updated

- You forgot to source the [uav_core shell additions](https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/shell_additions/shell_additions.sh).
Check that your `~/.bashrc` file is [set up correctly](https://github.com/ctu-mrs/uav_core) and particularly that it contains 
```bash
source $HOME/git/uav_core/miscellaneous/shell_additions/shell_additions.sh
```
- You have to be in root of a Gitman-maintaned git repository (*uav_core*/*simulation*):
```bash
cd ~/git/uav_core
git pull
```

### `roslaunch`: cannot find package *\<package_name\>*
Is *\<package_name\>* within a catkin workspace and is the workspace built?
If yes, check that your `~/.bashrc` contains
```bash
source /home/mrs/mrs_workspace/devel/setup.zsh
```

### Missing dependencies
Rerun the installation scripts of [uav_core](https://github.com/ctu-mrs/uav_core/blob/master/installation/install.sh) and [simulation](https://github.com/ctu-mrs/simulation/blob/master/installation/install.sh).
