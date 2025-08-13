---
title: ROS messages
pagination_label: ROS message defined by the MRS
description: ROS message defined by the MRS
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# MRS ROS messages

We keep all of our messages defined within a handful of packages.
This is important for replay-ability of old rosbags, which requires a compiled definition of ROS messages from the time of the rosbag recording.
Keeping all messages within a single package makes this process significantly easier (and possible) comparing to defining them across the whole system.

## Core message

<Button label="🔗 mrs_msgs API" link="https://ctu-mrs.github.io/mrs_msgs/" block /><br />

<Button label="🔗 mrs_msgs repository" link="https://github.com/ctu-mrs/mrs_msgs" block /><br />

## Modules message

<Button label="🔗 mrs_modules_msgs API" link="https://ctu-mrs.github.io/mrs_modules_msgs/" block /><br />

<Button label="🔗 mrs_modules_msgs repository" link="https://github.com/ctu-mrs/mrs_modules_msgs" block /><br />
