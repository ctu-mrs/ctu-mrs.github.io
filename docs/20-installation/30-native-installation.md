---
title: Native installation
---

# Native installation of the MRS UAV System

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

