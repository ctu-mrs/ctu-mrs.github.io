---
title: VINS IMU filter
pagination_label: VINS IMU filter
description: VINS IMU filter
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# VINS IMU filter

Provides filtering of IMU data to be used in VIO/VINS. Provides Butterworth low-pass filter and notch filters, configured separately for accelerometer and gyroscope data. Provides joining of separate accelerometer and gyroscope messages in case of the RealSense T256 IMU by copying the latest accelerometer message.

## Usage

To use with the ICM-42688 IMU:
```
ros 2 launch mrs_vins_imu_filter filter_icm_42688.py
```

To use with the RealSense T265 IMU:
```
ros 2 launch mrs_vins_imu_filter filter_t265.py
```

## Configuration
Edit / create configs in `config/` folder.

## Source files

The source code of the IMU filter can be found on [GitHub](https://github.com/ctu-mrs/mrs_vins_imu_filter/tree/ros2)
