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

Clone the [mrs_docker](https://github.com/ctu-mrs/mrs_docker) repository, then under [deployment/ros2/lazydocker](https://github.com/ctu-mrs/mrs_docker/tree/master/deployment/ros2/lazydocker) you'll find `tmux.sh`, `session.yml` and a [uav](https://github.com/ctu-mrs/mrs_docker/tree/master/deployment/ros2/lazydocker/uav) folder representing a specific drone configuration.

## Configure environment variables

The values in the `uav/stack.env` file will be set in every service from `compose.yaml`, this is set as an argument to docker compose in the `up.sh` and `down.sh` wrapper scripts.

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

Edit the `session.yml` file and change `DOCKER_HOST` and the rest of the arguments to the hostname of your drone and then run `./tmux.sh`. Alternatively, you can run it without tmux like `export DOCKER_HOST=tcp://uav30:2375 && ./up.sh`

If you wish, you can add more windows and control multiple drones with different `up.sh` scripts. After starting tmux, you'll get an overview of the session with lazydocker and the uav status, courtesy of [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status/tree/ros2).

To exit, navigate to the `compose` tab, run the `./down.sh` script and then hit the killing shortcut: `ctrl+a k` (`ctrl+a` and then `k`). A menu will appear in which you confirm the selection. It's possible to leave the containers running in the background while tmux is not running.

### Useful info

When running lazydocker, you may notice that an init container has exited, this is expected for the system to run properly, the custom node container will also exit unless you have built a colcon workspace into it.

We define the init service in `compose.yaml` and run it before all others for the purpose of creating volumes and filling them with shared data with a command like:

```bash
docker compose cp . init:/etc/docker
```

Using `docker compose up` will skip these steps so refrain from using it unless you modify the system to work without `up.sh`.

When using `DOCKER_HOST`, the image you run will need to be available on the drone, in case you need to transfer an image from your host, you can setup a [local registry](https://ctu-mrs.github.io/docs/prerequisites/docker/registries#using-a-local-docker-registry) or run a command like:

```bash
docker save ctumrs/mrs_uav_system:stable | docker -H tcp://uav30:2375 load
```
