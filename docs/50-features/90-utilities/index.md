---
title: ROS utilities
pagination_label: MRS ROS utilities
description: MRS ROS utilities
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# MRS ROS utilities

MRS Utilities contains various useful tools for development and testing.

<Button label="🔗 mrs_utils repository" link="https://github.com/ctu-mrs/mrs_utils" block /><br />

## mrs_tf_reconfigure

This tool will help you to construct transformations, which are needed to e.g. introduce a new sensor to your drone. These transformations can be hard to work out only by hand, especially if the sensor is tilted in multiple axes.
To further confuse you, Gazebo and ros tfs use different angle conventions, so you need different values for each of them.

Tf_reconfigure makes this process much easier, it will allow you to construct the final transformation by chaining multiple simpler transformations, and it will output the final translation and angles for gazebo and for ros tfs.

### running tf_reconfigure

Navigate to the start folder and run the tf_reconfigure.sh script. This will start a new tmux session, run rviz, and rqt_reconfigure. You can see your current transformation in rviz, and adjust the values for up to three chained simple transformations in the rqt_reconfigure window. At first, only the first chained transformation is visible, others will appear once you modify their values. Once you have configured your transformation, you can read out the final transformation in the terminal. Tf_reconfigure will output the final translation, quaternion, and angles for Gazebo and ros tfs.

[![](fig/tf_reconfigure.png "tf reconfigure")](fig/tf_reconfigure.png)

## mrs_camera_republisher

TODO

## mrs_circle_flier

TODO

## mrs_multireconfigure

TODO

## mrs_sensor_info

TODO

## mrs_tf_connector

TODO

## mrs_tf_estimator

A tool for calculating an affine transformation to align a trajectory measured in two different coordinate systems.

## mrs_tf_mirror

TODO

## mrs_throw_activation

TODO

