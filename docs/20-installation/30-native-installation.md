---
title: Native installation
pagination_label: Installing the MRS nativelly
description: Installing the MRS nativelly
---

# Native installation of the MRS UAV System

## Native installation

1. Install the Robot Operating System (Noetic):
```bash
curl https://ctu-mrs.github.io/ppa-stable/add_ros_ppa.sh | bash
sudo apt install ros-noetic-desktop-full
```

2. Configure your ROS environment according to [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3. Add the **[stable](https://github.com/ctu-mrs/ppa-stable)** PPA into your apt-get repository:
```bash
curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash
```
  * <details>
    <summary> Special instructions for the MRS System developers </summary>

      * Instead of the stable PPA, you can add the **[unstable](https://github.com/ctu-mrs/ppa-unstable)** PPA, for which the packages are build immediatelly after being pushed to **master**.
      * If you have both PPAs, the **unstable** has a priority.
      * Beware! The **unstable** PPA might be internally inconsistent, buggy and dangerous!

    </details>

4. Install the MRS UAV System:
```bash
sudo apt install ros-noetic-mrs-uav-system-full
```

5. Start the example Gazebo simulation session:
```bash
roscd mrs_uav_gazebo_simulation/tmux/one_drone
./start.sh
```

## How to update the MRS UAV System

Issue the following command to update the MRS UAV System:
```bash
sudo apt update && rosdep update && sudo apt upgrade --with-new-pkgs --allow-downgrades
```

### What does it do?

* `sudo apt update` will refresh the available debian packages from the archives.
* `rosdep update` will refresh the rosdep depedency tree >
* `sudo apt upgrade` will download the new versions of packages.
  * `--with-new-pkgs` will ensure that if new dependency appears for an already installed package, it will be installed as well.
  * `--allow-downgrades` will allow to downgrade a package version. This happens when installing our forks of official ROS packages.
