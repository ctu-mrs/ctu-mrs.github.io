---
title: Native installation
pagination_label: Installing the MRS natively
description: Installing the MRS natively
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Native installation of the MRS UAV System

## Native installation

1. Install the Robot Operating System (Jazzy):
```bash
curl https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash
sudo apt install ros-jazzy-desktop-full
```

2. Configure your ROS environment according to [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3. Add the **[stable](https://github.com/ctu-mrs/ppa2-stable)** PPA into your apt-get repository:
```bash
curl https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash
```
  * <details>
    <summary> Special instructions for the MRS System developers </summary>

      * Instead of the stable PPA, you can add the **[unstable](https://github.com/ctu-mrs/ppa-unstable)** PPA, for which the packages are build immediately after being pushed to **master**.
      * If you have both PPAs, the **unstable** has a priority.
      * Beware! The **unstable** PPA might be internally inconsistent, buggy and dangerous!

    </details>

4. Install the MRS UAV System:
```bash
sudo apt install ros-jazzy-mrs-uav-system-full
```

5. Set Zenoh to be the used RMW implementation. The Zenoh RMW is used by default in our example simulation sessions.
Add to `~/.bashrc` (`~/.zshrc`):
```
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
```

Source `~/.bashrc` (`~/.zshrc`):
```bash
source ~/.bashrc
```

6. Start the example simulation session:
```bash
cd /opt/ros/jazzy/share/mrs_multirotor_simulator/tmux/mrs_one_drone
./start.sh
```

## How to update the MRS UAV System

Issue the following command to update the MRS UAV System:
```bash
sudo apt update && rosdep update && sudo apt upgrade --with-new-pkgs --allow-downgrades
```

### What does it do?

* `sudo apt update` will refresh the available debian packages from the archives.
* `rosdep update` will refresh the rosdep dependency tree >
* `sudo apt upgrade` will download the new versions of packages.
  * `--with-new-pkgs` will ensure that if new dependency appears for an already installed package, it will be installed as well.
  * `--allow-downgrades` will allow to downgrade a package version. This happens when installing our forks of official ROS packages.
