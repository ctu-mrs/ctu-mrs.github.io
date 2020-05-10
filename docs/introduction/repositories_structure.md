---
layout: default
title: Repositories structure
parent: Introduction
nav_order: 99
---

# MAVROS topics and services

Following topics and services are used by the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system) to interact with [Mavros](https://github.com/mavlink/mavros).

## Services

  * *mavros/cmd/arming* - called to arm/disarm the vehicle
    * only disarming is used on the hardware, the software should not be able to arm the drone in normal "lab" use
  * *mavros/set_mode* - called to switch the PixHawk`s offboard mode on/off
    * this is used only in simulation, the software should not be able to trigger offboard mode in normal "lab" use
  * *mavros/cmd/command* - called to disarm the vehicle mid-flight

## Topics

### We subscribe to

  * *mavros/rc/in* - reports on the RC channels
    * this topic is not vital, it is used to trigger safety landing by a safety pilot
  * *mavros/state* - reports on the current state of PixHawk: armed/disarmd, offboard, etc.
    * this topic is required for takeoff 
  * *mavros/local_position/odom* - publishes PixHawk's state estimate
    * this topic is required to initialize the sensor fusion
  * *mavros/global_position/global* - publishes raw GPS data
    * when GPS is used in the fusion, this topic is required to initialize the fusion
  * *mavros/imu/...* - publishes IMU
    * this topic is not vital
  * *mavros/battery/...* - publishes battery state
    * this topic is not vital and can be ommited

### We publish on

  * *mavros/setpoint_raw/attitude* - contains the desired attitude rate and thrust for controlling the UAV in the offboard mode
