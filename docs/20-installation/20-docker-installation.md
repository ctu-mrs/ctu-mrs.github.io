---
title: Docker installation
pagination_label: Installing the MRS using Docker
description: Installing the MRS using Docker
---

# Docker installation

The MRS System is provided in the form of pre-compiled docker images on [Dockerhub](https://hub.docker.com/u/ctumrs).

## Rolling stable version

The latest image is available at

```yaml
ctumrs/mrs_uav_system:stable
```

You can plug this image into [rosker](../10-prerequisites/25-ros2/10-installation.md#docker) from the ROS2 Installation guide by changing these two variables like so:

```bash
rosker() {
  local name="${1:-stable}"
  local image="ctumrs/mrs_uav_system:$name"
```

Then run it with:

```bash
docker pull ctumrs/mrs_uav_system:stable
rosker
```
