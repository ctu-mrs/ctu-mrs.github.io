---
title: Standalone Docker
pagination_label: Deploying using standalone Docker
description: Deploying using standalone Docker
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Deployment using standalone Docker

Docker is a platform that allows you to build, share, and run applications quickly in a repeatable and isolated way. It uses containers which are like extended versions of chroot.

This page will describe how to run a Docker Compose session of the MRS UAV System using Lazydocker and Tmux. For more info about what those are and how to install them, please check the [prerequisites](https://ctu-mrs.github.io/docs/prerequisites/).

## Get the mrs_docker repo

Clone the [mrs_docker](https://github.com/ctu-mrs/mrs_docker) repository, then under [deployment/ros2/lazydocker](https://github.com/ctu-mrs/mrs_docker/tree/master/deployment/ros2/lazydocker) you'll find `tmux.sh`, `session.yml` and folders representing a specific drone configuration.

In this example, we'll use [uav30](https://github.com/ctu-mrs/mrs_docker/tree/master/deployment/ros2/lazydocker/uav30) with `up.sh` and `down.sh` wrappers for docker commands.

## Configure environment variables 

The values in the `stack.env` file will be set in every service from `compose.yaml`, this is set as an argument to docker compose in `up.sh` and `down.sh`.

For more info about setting the environment variables, check the [native installation](https://ctu-mrs.github.io/docs/deployment/native/bashrc_configuration#bashrc-for-a-real-uav).

## Start the compose session

Before you start the session, you should edit `compose.yaml` according to your needs, for example to add or remove sensors your drone uses. The main image used will be [ctumrs/mrs_uav_system](https://hub.docker.com/r/ctumrs/mrs_uav_system).

### On the drone

You can use `./up.sh` to start a session in the background and `./down.sh` to end it.

Running `./up.sh -h` will show you the arguments that can be passed to the script, notably `-n` which will make the script not try to overwrite colcon_workspace.

These scripts source `setup.sh`, in it the `SESSION_NAME` variable is set which will prefix every resource created by the compose session like volumes and containers:

```bash
just_flying_shared_data
just_flying_zenoh
```

### Remote

For this to work, the Docker daemon on the drone must be configured according to the instructions at [docker-host](https://ctu-mrs.github.io/docs/prerequisites/docker/docker-host), we also recommend this [ssh configuration](https://ctu-mrs.github.io/docs/prerequisites/ssh).

Edit the `session.yml` file and change `DOCKER_HOST` and the rest of the arguments to the hostname of your drone and then run `./tmux.sh`.

If you wish, you can add more windows and control multiple drones with different `up.sh` scripts. After starting tmux, you'll get an overview of the session with lazydocker and the uav status, courtesy of [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status/tree/ros2).

To exit, hit the killing shortcut: `ctrl+a k` (`ctrl+a` and then `k`). A menu will appear in which you confirm the selection.

### Useful info

When running lazydocker, you can see that an init container has exited, this is expected for the system to run properly.

We define the init service in `compose.yaml` and run it before all others for the purpose of creating volumes and filling them with shared data with a command like:

```bash
docker compose cp . init:/etc/docker
```

Using `docker compose up` will skip these steps so please refrain from using it unless you customize the system to work without `up.sh`.
