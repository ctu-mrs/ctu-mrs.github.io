# File structure

After the installation (see [uav_core/README.md](https://mrs.felk.cvut.cz/gitlab/uav/uav_core)), several directories are created in your **home** folder. Our main repositories were cloned to **~/git**

# ~/git

```
~/git
  ├── uav_core
  ├── uav_modules
  ├── simulation
  └── linux-setup
```

**~/git/[uav_core](uav_core)** contains "the core" of our packages, literally.
Those are minimal required for the drone to fly.

**~/git/[uav_modules](uav_modules)** contains optional packages, which might be useful for working with drones.

**~/git/simulation** is necessary to run simulations. It also includes 3D models and worlds.

**~/git/[linux_setup](http://github.com/klaxalk/linux-setup)** sets up **tmux**, **vim** and bash environment.
Usually, there is no need to modify anything in this repository, just keep it updated by occasional **git pull**.

# ~/mrs_workspace

mrs_workspace is a ROS workspace which contains both our repositories (linked from ~/git).
Keep them updated and keep the workspace [compiled](how_to_compile).
Packages in [uav_core](uav_core) and [uav_modules](uav_modules) are not meant to be modified unless you work on some core feature in them.
In that case, you will be instructed.

```
~/mrs_workspace
  ├── build
  ├── devel
  ├── logs
  └── src
      ├── uav_core -> ~/git/uav_core
      └── uav_modules -> ~/git/uav_modules
```

# ~/workspace

This ROS workspace is meant for **your** personal code.
For a start, the only packages inside are from the **templates** repository, which should help you start with your code.

```
~/workspace
  ├── build
  ├── devel
  ├── logs
  └── src
      └── templates
```
