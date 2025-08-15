---
title: Localization
pagination_label: Localization sources within the MRS
description: Localization sources within the MRS
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Localization sources within the MRS

* TODO explain the reason for the need of localization
* TODO explain that the sources of localizaton are fused by the estimation manager
* TODO point to the metapackages responsible for defining the estimator plugin and the other dependencies

## GNSS + Magnetometer

* TODO State estimator is part of the MRS core.
* TODO Direct integration with PX4 over the HW API.

## RTK GNSS

* TODO State estimator is part of the MRS core.
* TODO Direct integration with Emlid Reach.

## Hector SLAM

* TODO Is prebuilt and part of the MRS

## ALOAM SLAM

* TODO Is prebuilt and part of the MRS

## OpenVINS Visual Inertial Odometry (VIO)

* TODO Is prebuilt and part of the MRS

## PointLIO SLAM

* TODO the estimator plugin is prebuilt
* TODO the SLAM is not part of the MRS

## LIOSAM SLAM

* TODO The SLAM and the estimator plugins are prebuilt in the MRS but not test in the realworld
