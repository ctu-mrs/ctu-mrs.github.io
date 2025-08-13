---
title: Apptainer installation
pagination_label: Installing the MRS using Apptainer
description: Installing the MRS using Apptainer
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Apptainer

[Apptainer](https://apptainer.org/) is an open-source **containerization** software designed specifically for high-performance computing (HPC), scientific research, and environments requiring strong security.
Apptainer enables users to create, manage, and run containers that package applications with their dependencies in a portable, reproducible, and isolated manner.
It was originally known as [Singularity](https://sylabs.io/singularity/) and is popular in academic and research institutions due to its focus on HPC environments.

## Relation to Singularity

Apptainer is the continuation of [Singularity](https://sylabs.io/singularity/) under a new governance model.
In 2021, the Singularity project moved under the Linux Foundation, where it was rebranded as Apptainer to ensure the long-term sustainability and broader collaboration around the project.
Functionally, Apptainer continues Singularity’s legacy, maintaining compatibility with its containers and focusing on the same core goals.

## Using Apptainer via wrapper repository

Please follow this link to the [repository](https://github.com/ctu-mrs/mrs_apptainer) container the Apptainer wrapper, configs and build scripts.

<Button label="🔗 ctu-mrs/mrs_apptainer repository" link="https://github.com/ctu-mrs/mrs_apptainer" block /><br />

## Doing it yourself

Feel free to learn and master Apptainer yourself with the help of the official [documentation](https://apptainer.org/documentation/).
Your first apptainer image of the MRS UAV System can be built by bootstrapping from our docker image.
You can bootstrap from the latest _rolling_ version:
```
ctumrs/mrs_uav_system:latest
```
or from a particular version, e.g.,
```
ctumrs/mrs_uav_system:1.5.0
```
