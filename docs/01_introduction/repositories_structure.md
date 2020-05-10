---
layout: default
title: Repositories structure
parent: Introduction
grand_parent: Introduction
---

# Suggested reading

 * Baca, IROS 2018, MPC tracker: [http://mrs.felk.cvut.cz/data/papers/iros_2018_mpc.pdf](http://mrs.felk.cvut.cz/data/papers/iros_2018_mpc.pdf)
   * brief description of the control pipeline
 * Hert, Diploma Thesis, [https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y](https://dspace.cvut.cz/bitstream/handle/10467/76157/F3-DP-2018-Hert-Daniel-thesis_hertdani.pdf?sequence=-1&isAllowed=y)
   * Improvements of the MPC tracker
   * Nice showcase of how to use our system
 * Our wiki for newcomers: [https://mrs.felk.c/gitlab/uav/uav_core/wikis/home](https://mrs.felk.cvut.cz/gitlab/uav/uav_core/wikis/home)
   * contains tutorials for basic manipulation with simulation
 * Lee, 2011, SO(3) control: [https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5717652](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5717652)
   * the non-linear state feedback used in our control pipeline

# Repositories

## uav_core

  * Contains submodules for the "necessary" parts of the control and estimation pipeline.
  * URL: [http://mrs.felk.cvut.cz/gitlab/uav/uav_core](http://mrs.felk.cvut.cz/gitlab/uav/uav_core)

### mavros - an interface between ROS and MavLink

### mrs_uav_manager - contains "managers"

  * **ControlManager** - receives references and state estimate, manages controllers and trajectory trackers, implements low-level safety mechanisms
  * **UavManager** - handles "high-level" tasks, such as takeoff and landing state machine, and higher-level safety features
  * **ConstraintManager** - constraint management for trackers
  * **GainManager** - gain management for the SO(3) controller

### mrs_controllers - controller plugins for the ControlManager

  * **So3Controller** ([link](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5717652)) for normal/faster flight
  * **MpcController** for a slower/robust flight
  * **FailsafeController** controller for feedforward landing in emergancy
  * **NsfController** - nonlinear state feedback, simpler than SO(3), uses Euler angles, more of an example of a UAV controller

### mrs_trackers - tracker plugins for the ControlManager

  * **MpcTracker** ([link](http://mrs.felk.cvut.cz/data/papers/iros_2018_mpc.pdf)) - constrained trajectory tracking, multi-UAV collision avoidance, safety area
  * **LineTracker** - simple and naive "goto" tracker, never used directly, just as a "template"
  * **LandoffTracker** - takeoff and landing, built upon the LineTracker
  * **JoyTracker** - Joystick attitude commands from a pilot
  * **MatlabTracker** - handoff to Matlab Control Toolbox
  * **CsvTracker** - tracking of pre-calculated trajectories in CSV (when exact tracking is required)

### mrs_odometry - sensor fusion

  * acquires sensor data and produces custom UAV state estimate

### mrs_optic_flow - MRS implementation of monocular optic flow

### mrs_mavros_interface - provides some abstraction above Mavros

  * interprets Mavros's heartbeat

### mrs_lib - custom MRS libraries and support code

  * custom **UKF** and **LKF** implementation
  * **ParamLoader** for convenient parameter loading in ROS
  * **SubscriberHandler** for convenient subscriber and callback management
  * **Transformer** for convenient ROS transformations
  * **AttitudeConverter** for convenient conversions between various orientation representations
  * **DynamicReconfigureMgr** for convenient dynamic reconfigure server management

... and much more

### mrs_status - terminal UAV status monitor and watchdog

### mrs_general - misc configurations and minor utilities

  * camera calibrations
  * flight area definitions
  * automatic start routine

### mrs_msgs - custom ROS messages used by the MRS software

* Each custom message should be defined here, in this isolated package. Future playback of old rosbags is then dependent only on this package, which is easy to compile without dependencies.

## uav_modules

  * Contains non-essential packages, e.g., sensor drivers and hardware-specific software.
  * URL: [http://mrs.felk.cvut.cz/gitlab/uav/uav_modules](http://mrs.felk.cvut.cz/gitlab/uav/uav_modules)

## simulation

  * Gazebo/ROS simulation pipeline.
  * Includes the PX4 Firmware, 3D models for Gazebo and Gazebo plugins (virtual sensors)
  * URL: [http://mrs.felk.cvut.cz/gitlab/uav/simulation](http://mrs.felk.cvut.cz/gitlab/uav/simulation)

# MAVROS topics and services

Following topics and services are used by the MRS stack to interact with MAVROS.

## Services

  * *mavros/cmd/arming* - called to arm/disarm the vehicle
    * only disarming is used on the hardware, the software should not be able to arm the drone in normal "lab" use
  * *mavros/set_mode* - called to switch the PixHawk`s offboard mode on/off
    * this is used only in simulation, the software should not be able to trigger offboard mode in normal "lab" use

## Topics

### We subscribe to

  * *mavros/setpoint_raw/target_attitude* - reports on current internal reference for the PixHawk's controllers
    * this topic is required, however, it starts to publish only after takeoff
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

### We publish

  * *mavros/setpoint_raw/attitude* - contains the desired attitude/rates for controlling the UAV in the offboard mode

# Running the control pipeline

  * when simulation, the scripts in `~/git/simulation/start_simulation/` are used to spawn a TMUX session with pre-defined commands
  * when flying with hardware, the scripts in `~/git/uav_core/tmux_scripts/` are used to spawn a TMUX session with pre-defined commands

  All ROS nodes in the control pipeline are started under one *nodelet manager* to minimized transport delays.
  The launch file which aggregates the control pipeline can be found under **mrs_uav_manager/launch/**.
