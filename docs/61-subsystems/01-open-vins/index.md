---
title: OpenVINS
pagination_label: OpenVINS
description: OpenVINS
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# OpenVINS

This folder contains docs on the subsystems needed to run VIO, specifically OpenVINS, with the MRS UAV System.

import DocCardList from '@theme/DocCardList';

<DocCardList />

To run [Bluefox2](https://ctu-mrs.github.io/docs/sensors/bluefox2) and [Bluefox2 IMU](https://ctu-mrs.github.io/docs/sensors/bluefox2-imu) in the same service, place the following compose session snippet into the [example session](https://ctu-mrs.github.io/docs/deployment/docker/):

```yaml
  vio:
    image: ctumrs/bluefox2:stable
    network_mode: host
    volumes:
      - /dev/:/dev/
    privileged: true
    env_file:
      - ./stack.env
    entrypoint: [
      "bash", "-c",
      "source /opt/ros/jazzy/setup.sh && ros2 launch mrs_open_vins_core vio_real.launch.py enable_bluefox_cam_and_imu:=true"
    ]
```
