---
title: Kalibr
pagination_label: Camera calibration using Kalibr
description: Camera calibration using Kalibr
---

# Camera calibration using Kalibr

We often utilize the [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) for camera calibration.

## Docker image and wrapper scripts

The [ctu-mrs/kalibr_docker](https://github.com/ctu-mrs/kalibr_docker) repository provides a convenient `compose_session` which utilizes a custom docker image.
Therefore, there is no need to compile kalibr manually (which would consume approx. 5 GB of HDD space).
The Kalibr docker image is sourced from the `ctumrs` [dockerhub](https://hub.docker.com/u/ctumrs).

<Button label="🔗 ctu-mrs/kalibr_docker repository" link="https://github.com/ctu-mrs/kalibr_docker" block /><br />

## Howto

0. If you have bag file from ROS2, you need to first [convert it to the ROS1 bag format](../120-miscellaneous/bagfile_convert_ros1_ros2.md)


1. clone the [ctu-mrs/kalibr_docker](https://github.com/ctu-mrs/kalibr_docker) repository
2. `cd compose_session`
2. Pull the docker image by calling `docker compose pull`
3. Store your bag file with the camera topic in the `data` folder and retrieve the results there afterwards.
4. Configure the parameters by editing the `compose.yaml` file.
5. Use `run.sh` to start the `compose` session with kalibr.

## Output conversion to the `camera_calibration` format

Use the script in `output_conversion` to convert the Kalibr format (with the `radtan` model) to the output of `camera_calibration`.
