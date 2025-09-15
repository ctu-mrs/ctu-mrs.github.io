---
title: Estimator plugins
pagination_label: API for the estimator plugins
description: API for the estimator plugins
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::


# The Estimator plugin interface
The Estimator plugin takes in IMU sensor data(Orientation and Angular velocity) and Controller command and provides UAV state consisting of position, velocity and acceleration. Diagram below shows the heirarchical structure defining base classes and derived classes.

The estimator plugin is compiled as ROS plugins with the interface defined by the estimator manager. A estimator plugin from any ROS package can be loaded dynamically by the estimator manager without it being present during estimator manager's compile time. Loaded estimators can be switched by the estimator manager in mid-flight, which allows safe testing of new estimator and adds flexibility to the MRS UAV system.

![estimator_dependency_tree-Page-1.drawio](https://hackmd.io/_uploads/BJq_KRBolx.png)

# Estimator Output
* Position
* Velocity
* Acceleration

# Example Estimator Plugin


* The estimation manager loads parameters from `custom_config.yaml` 
* The MRS system provides different levels of abstraction for developing estimator plugins. 

* Partial estimator: Partial estimators provide abstract class for partial estimation. 

* Generic estimators for individual states like altitude, lateral and heading are already implemented. 

* These partial estimators are recommended to be customised by appropriate modification of parameters in `custom_config.yaml`.

  TODO::// Description of config file

<!-- * StateGeneric is the final abstraction class, it inherits from state estimator. 
Required partial estimators are initialised here.

* StateGeneric is suppose to act like an final abstract class, which can be inherited and used to configure for custom hardware used. Mainly we do modifications in `custom_config.yaml` file to achieve this.

* Estimation is always in perticular frame of reference. estimators are capable of doing transformations between frames but it is necessory to mention the required frame in the format given in yaml file. More information about frames of reference can be found [here](https://ctu-mrs.github.io/docs/api/frames_of_reference)

* Estimator is mentioned at different managers like uav_manager, gain_manager and transformation_manager. It is necessory that these are configured properly with modifications to custom parameters in `custom_config.yaml` file. -->
