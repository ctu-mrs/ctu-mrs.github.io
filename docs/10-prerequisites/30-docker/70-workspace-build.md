---
title: ROS Workspace build
pagination_label: Building and caching of ROS workspace in Docker
description: How to build and cache a ROS workspace into a Docker image
---

# ROS Workspace caching with Docker

This page describes the process of compiling **custom ROS packages** against dependencies from **base docker image**. This task is not trivial in general, however, we have prepared a set of scripts that will make the process straightforward. The following **assumption** apply for our situation:

* We have a **base docker image**, which covers our dependencies, e.g., `ctumrs/mrs_uav_system:1.5.0`,
* We need to **cache the build artifacts** (`./build`, `./devel`, `./.catkin_tools` within the workspace) for future rebuilds of the same software.
* We need to transport the resulting compiled software to an offline machine (a.k.a. the **robot**),
* The **robot** has the **base docker image** loaded in its docker daemon.

Our process comprises of the following docker build stages:

1. **Stage 1: build the workspace**
    * Load the build cache (`./cache`) from the previous build
    * Compile the catkin workspace
2. **Stage 2: save the cache**
    * Encapsulation of the whole workspace into a transport image (`alpine:latest`)
    * Export of the image into a local directory (`./cache`)
3. **Stage 3: export the transport image**
    * Copy of the build necessary build artifacts into a transport image (`alpine:latest`)
    * Export of the transport image

The compiled workspace is **transported** into the **robot** within an minimalistic `alpine`-based image, which makes it relatively small. The overhead of the transport image is only around 5 MB. On the other hand, if the workspace would be packed in a image based on the **base image**, the size would be offset by hundreds of megabytes. 

That is not a problem when the transport occurs through a **docker registry**. However, since the [Portainer](../32-portainer/index.md) interface makes the upload of **archived** images very simple, we prefer to bundle the whole image into a `.tar.gz` file. This approach complicates the deployment in one simple way: The workspace needs to **extracted** from the transport image and placed into a **shared volume** during runtime.

## Pre-configured build pipeline

A set of scripts that facilitate the build is provided at [ctu-mrs/mrs_docker](https://github.com/ctu-mrs/mrs_docker) under the `catkin_workspace_builder` folder.

<Button label="🔗 ctu-mrs/mrs_docker repository" link="https://github.com/ctu-mrs/mrs_docker" block /><br />

### Configuration

* The sources of your ROS packages are supposed to be placed (or linked) into the `./src` folder.
* The file `./common_vars.sh` configures the process by setting up the following environment variables:

```bash
# tag for the source image against which the catkin workspace will be built
export BASE_IMAGE=ctumrs/mrs_uav_system:1.5.0

# tag for the 'transport' image used for packing the workspace
export TRANSPORT_IMAGE=alpine:latest

# tag for the resulting image in which the workspace will be packaged
export OUTPUT_IMAGE=catkin_workspace:1.0.0

# location for the exported docker images using the `./export_image.sh` script
export EXPORT_PATH=~/docker

# CPU architecture for the output image
export ARCH=amd64

# path to the catkin workspace in the docker image
export WORKSPACE_PATH=etc/docker/catkin_workspace

# local path to the build cache
export CACHE_PATH=cache
```

### Execution

* Run `./build_image.sh` to compile the ROS packages into the image.
* Run `./export_image.sh` to export the transport image with the workspace to the export destination.
* Run `./clean.sh` to clean the build cache.

## Using the workspace on a robot

The ROS colcon workspace needs to be first extracted from the **transport image** and copied into a shared docker volume. Then, a container can be started using the **base image** and the workspace can be sourced and used to start your packages.

Here is an example of a minimalistic compose session.
The full example can be found at [ctu-mrs/mrs_docker/compose/custom_workspace](https://github.com/ctu-mrs/mrs_docker/compose/custom_workspace)

```yaml
volumes:

  catkin_workspace:

services:

  # will copy user's ROS catkin workspace from the 'transport' alpine image to a shared volume
  copy_catkin_workspace:
    image: <TRANSPORT_IMAGE>
    volumes:
      - catkin_workspace:/tmp/docker/catkin_workspace:consistent
    tty: true
    command: sh -c "rm -rvf /tmp/docker/catkin_workspace/*; mkdir -pv /tmp/docker/catkin_workspace; cp -rv /etc/docker/catkin_workspace/* /tmp/docker/catkin_workspace/"

  # starts the HW API for connecting the MRS UAV System to PX4
  custom_package:
    image: <BASE_IMAGE>
    network_mode: host
    depends_on:
      - copy_catkin_workspace
    volumes:
      - catkin_workspace:/etc/docker/catkin_workspace:consistent
    env_file:
      - ./stack.env
    tty: true
    command: bash -c "source /etc/docker/catkin_workspace/devel/setup.bash && waitForRos && roslaunch <my_package> <my_launchfile>.launch"
```
