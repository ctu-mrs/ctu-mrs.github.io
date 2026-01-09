---
title: Server
pagination_label: Octomap Server
description: Octomap Server
---

# Octomap server

<Button label="🔗 mrs_octomap_server repository" link="https://github.com/ctu-mrs/mrs_octomap_server" block /><br />

3D occupancy mapping based on [Octomap](https://octomap.github.io/). The sensor data are inserted into a local map centered at the current position of the robot. The local maps are periodically copied into a global map. The map can incorporate pointclouds from 3D LiDARs and depth cameras.

## Usage
See [octomap_mapping_planning launch file](https://github.com/ctu-mrs/mrs_octomap_mapping_planning/blob/master/ros_packages/mrs_octomap_mapping_planning/launch/mapplan.launch) for setting of relevant topics, passing a custom config to the octomap server and setting its parameters.

## Sensor topics
Topic names are set as arguments to the [launch file](https://github.com/ctu-mrs/mrs_octomap_server/blob/master/launch/octomap.launch).
| Topic name                                 | Type                    | Meaning                                                                          |
|--------------------------------------------|-------------------------|----------------------------------------------------------------------------------|
| lidar_3d_topic_*NUM*_in                    | sensor_msgs/PointCloud2 | 3D LiDAR pointcloud                                                              |
| lidar_3d_topic_*NUM*_over_max_range_in     | sensor_msgs/PointCloud2 | points over max range (will be used for free-space raycasting only               |
| depth_camera_topic_*NUM*_in                | sensor_msgs/PointCloud2 | depth camera pointcloud                                                          |
| depth_camera_topic_*NUM*_over_max_range_in | sensor_msgs/PointCloud2 | depth camera points over max range (will be used for free-space raycasting only  |
| camera_info_topic_*NUM*_in                 | sensor_msgs/CameraInfo  | depth camera info (used for calculating free-space raycasting parameters)        |

## Configuration
The frame of reference, where the map is built, is set by argument **world_frame_id** of the launch file.
Default configuration parameters are available at [the config file](https://github.com/ctu-mrs/mrs_octomap_server/blob/master/config/default.yaml). They can be overridden by passing a custom config file to the main launch file.
