---
title: Apptainer
pagination_label: Deploying using Apptainer
description: Deploying using Apptainer
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Deployment using Apptainer

Apptainer is an open source container platform designed to run complex applications on high-performance computing (HPC) clusters.

## Useful info

To transfer images between Docker and Apptainer, you have two options, you can save and build it:

```bash
docker save ctumrs/mrs_uav_system:stable > mrs.tar
apptainer build mrs.sif docker-archive:mrs.tar
```

Or you can setup a [local registry](https://ctu-mrs.github.io/docs/prerequisites/docker/registries#using-a-local-docker-registry) in Docker and pull the image into Apptainer:

```bash
apptainer pull mrs.sif localhost:5000/ctumrs/mrs_uav_system:latest
```
