---
title: Installation
pagination_label: Installing ROS2 Jazzy
description: Installing ROS2 Jazzy
---

# Installing ROS2 Jazzy

The following commands will install ROS2 Jazzy on Ubuntu 24.04:
```bash
sudo apt-get -y install software-properties-common curl bash

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get -y update

sudo apt-get -y install ros-jazzy-desktop-full ros-dev-tools
```

## Using MRS's script

Alternatively, you can use our setup script for adding the ROS PPA:
```bash
sudo apt-get -y install software-properties-common curl bash

curl https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash

sudo apt-get -y install ros-jazzy-desktop-full ros-dev-tools
```

## Docker

If you don't want to run ROS natively, you can run it in a container with the VS Code [dev containers](https://code.visualstudio.com/docs/devcontainers/containers) extension or you can place the following helper function in your `~/.bashrc` which will help manage permanent containers based on the MRS ROS images.

```bash
rosker() {
  local name="${1:-jazzy}"
  shift

  if docker ps -a --format '{{.Names}}' | grep -xq "$name"; then
    if docker ps --format '{{.Names}}' | grep -xq "$name"; then
      docker exec -it "$name" ./ros_entrypoint.sh bash -ic "${*:-exec bash}"
    else
      docker start -ai "$name"
    fi
  else
    docker run -it --name "$name" --network=host --privileged \
      -v "$HOME/:/root/" \
      -v "/dev:/dev/" \
      -v "/etc/hosts:/etc/hosts" \
      "ctumrs/ros_$name:latest" bash
  fi
}
```

It'll give you an interactive terminal no matter what state the container is (started, stopped, non-existent) or run a command if it's running, usage examples:

```
rosker
rosker jazzy
rosker jazzy ros2 topic list
````

Note that by default your home directory (and devices) will be available in the container, to avoid issues with `~/.bashrc` errors, you can conditionally source things like:

```bash
[ -f /root/git/mrs_uav_development/shell_additions/shell_additions.sh ] ||
source ~/git/mrs_uav_development/shell_additions/shell_additions.sh
```

Which will not source this file if it's located in /root (in the container), if you want to source it, you should change `||` for `&&`
