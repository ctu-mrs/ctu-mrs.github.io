---
title: Localization
pagination_label: Localization sources within the MRS
description: Localization sources within the MRS
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Localization sources within the MRS

## The Need for Localization

Accurate and reliable localization is the cornerstone of autonomous UAV operation. For a drone to navigate its environment, execute complex missions, and avoid obstacles, it must have a precise understanding of its own position and orientation (collectively known as *pose*). This awareness is fundamental for everything from simple waypoint following to advanced tasks like autonomous landing and interaction with the environment. Without a robust localization system, a UAV is essentially flying blind, posing a significant risk to itself and its surroundings.

### Localization Sources

The MRS UAV System is designed to be versatile and can utilize a variety of localization sources. These sources are fused by an estimation manager, which combines the data to produce a single, highly accurate state estimate. The primary sources can be broadly categorized into two groups: GNSS-aided and GNSS-denied.

#### GNSS-Aided Localization

When operating in open, outdoor environments, Global Navigation Satellite Systems (GNSS) are the most common source for localization. These systems provide a global position estimate with reasonable accuracy.

* **GNSS:** Basic GNSS receivers provide position information that can be fused with data from an Inertial Measurement Unit (IMU) and a magnetometer to get a full pose estimate.
* **RTK GNSS:** For applications requiring higher precision, Real-Time Kinematic (RTK) GNSS can provide centimeter-level accuracy by using a secondary base station to correct for satellite signal errors.


#### GNSS-Denied Localization

In many critical scenarios, relying on GNSS is not an option. This can be due to signal obstruction, such as in underground tunnels or dense urban canyons, or due to intentional interference like GPS jamming in security-sensitive missions. In these cases, the UAV must rely on its onboard sensors to estimate its motion. This is where GNSS-denied localization algorithms become essential.

The most common approaches are:

* **Visual-Inertial Odometry (VIO):** This technique uses one or more cameras combined with an IMU to track the drone's motion. VIO is a popular choice as cameras are lightweight, low-power, and relatively inexpensive. By tracking visual features in the environment from frame to frame, VIO algorithms can estimate the drone's ego-motion. However, they can be susceptible to drift over long distances and may struggle in visually-degraded environments (e.g., low light or textureless surfaces).
* **Lidar-Inertial Odometry (LIO):** LIO replaces cameras with a LiDAR sensor to build a point cloud of the environment. By matching consecutive point clouds, LIO systems can very accurately track the drone's movement. LIO is generally more robust than VIO in challenging lighting conditions and can provide higher accuracy. The tradeoff is that LiDAR sensors are typically more expensive, heavier, and consume more power, which can impact the drone's flight time and budget.
* **Simultaneous Localization and Mapping (SLAM):** Both VIO and LIO are often integral parts of a larger SLAM system. SLAM algorithms not only track the drone's pose but also build a persistent map of the environment. This map can then be used to correct for drift when the drone revisits a previously mapped area (an event known as a *loop closure*), leading to globally consistent trajectories.

The choice of a particular localization algorithm is a trade-off that depends on the specific application, the operational environment, and, significantly, the available budget. While a high-end LiDAR-based system might offer the best performance, a well-tuned VIO system can provide excellent results for many applications at a fraction of the cost.

## Estimation Manager and Source Fusion

The MRS estimation manager fuses multiple localization sources, including GNSS, VIO, and LIO, into a single, robust state estimate for the UAV. It manages switching between estimator plugins based on their health and publishes essential topics such as the UAV state, odometry, and height above terrain. Estimator plugins provide a standardized interface to expose different localization algorithms to the manager. For detailed information, see the [Estimation Manager documentation](../../50-features/01-managers/index.md#estimationmanager).

## Localization Integration Structure

<div style={{textAlign: 'center'}}>
  <img src={require('./fig/Algo_Integration_Pipeline_MRS.png').default} alt="MRS Localization Layers" style={{width: '35%'}} />
</div>

To integrate a localization algorithm into the MRS UAV System, the implementation follows a three-layer structure that bridges the raw algorithm to the system's estimation framework.

### Algorithm Library

The algorithm library contains the core implementation of the localization algorithm (for example, `open_vins` for OpenVINS). This layer includes all mathematical models, feature tracking, optimization, and pose estimation code, but it operates in its own coordinate frames and ROS topics, which are not directly compatible with the MRS UAV System.

### Core Folder

The core folder (for example, `mrs_open_vins_core`) acts as the ROS integration wrapper. It provides configuration files, launch files for node wrapping and topic remapping, and tmux scripts for simulation startup (including RViz visualization). This layer handles all MRS-specific wiring and supports both simulation and real-world deployment. It allows the [Algorithm Library](#algorithm-library) to remain untouched, while still integrating the algorithm into the MRS UAV System. For instructions on how to start the Gazebo simulation, see the [How to Simulate](./../../60-simulations/01-gazebo/01-howto.md) page.


### Estimator Plugin Folder

The estimator plugin folder (e.g., `mrs_open_vins_estimator_plugin`) implements the MRS estimator plugin interface, making the algorithm selectable by the estimation manager. See the [estimator plugins documentation](./../../45-plugin-interface/04-estimators.md) for full interface details.

## Optional Components

Algorithms that generate their own TF trees (such as VINS-based approaches) typically require a republisher node. This node is responsible for transforming the algorithm's odometry into the MRS `world` frame and linking the algorithm's internal TF tree to the main `fcu` frame via static transforms (specifically from `fcu` to the algorithm's `body` or `imu` frame). For a reference implementation, see the [VINS Republisher](https://github.com/ctu-mrs/mrs_vins_republisher.git).

For detailed information on the system's coordinate frame conventions, refer to the [Frames of Reference](./../../40-api/20-frames_of_reference.md) page.

## Available Localization Algorithms

Below you can find the localization algorithms currently integrated into the MRS UAV System. Refer to the documentation of each algorithm to learn about its installation, configuration, and usage within the estimation framework.

import DocCardList from '@theme/DocCardList';

<DocCardList />

