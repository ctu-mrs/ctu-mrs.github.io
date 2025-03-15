---
title: Installation
pagination_label: Installing ROS2 Jazzy
description: Installing ROS2 Jazzy
---

# Installing ROS2 Jazzy

The following commands will install ROS2 Jazzy on Ubuntu 24.04:
```bash
sudo apt-get -y update
sudo apt-get -y install curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get -y update

sudo apt-get -y install ros-jazzy-desktop-full ros-dev-tools
```

## Using MRS's script

Alternatively, you can use our setup script:
```bash
RUN apt-get -y install software-properties-common curl bash

RUN curl https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash
```
