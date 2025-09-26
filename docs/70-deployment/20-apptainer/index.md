---
title: Apptainer
pagination_label: Deploying using Apptainer
description: Deploying using Apptainer
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Deployment using Apptainer

Apptainer is an open source container platform designed to run complex high-performance computing (HPC) applications.

## Useful info

To transfer images between Docker and Apptainer, you have several options, you can save and build it:

```bash
docker save ctumrs/mrs_uav_system:stable > mrs.tar
apptainer build mrs.sif docker-archive:mrs.tar
```

You can also do that with no temporary file and remotely with SSH

```bash
ssh uav30 'apptainer build mrs.sif docker-daemon:ctumrs/mrs_uav_system:stable'
```

Or you can setup a [local registry](https://ctu-mrs.github.io/docs/prerequisites/docker/registries#using-a-local-docker-registry) in Docker, add it as an insecure registry

```bash
sudo tee /etc/containers/registries.conf.d/mrs.conf <<EOF
[[registry]]
location = "localhost:5000"
insecure = true
EOF
```

And pull the image into Apptainer:

```bash
apptainer pull mrs.sif docker://localhost:5000/ctumrs/mrs_uav_system:latest
```
