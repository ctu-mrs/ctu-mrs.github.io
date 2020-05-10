---
layout: default
title: Buttons
parent: Introduction
---

# uav_modules repository

uav_modules repository contains optional ROS packages.

``` 
~/git/uav_modules
├── README.md
└── ros_packages
    ├── apriltag2_ros
    ├── bluefox2                                   ( camera driver ) 
    ├── camera_base                                ( camera driver ) 
    ├── hector_mapping
    ├── mrs_auto_exposure
    ├── mrs_bumper
    ├── mrs_gazebo_ros_interface                   ( simulation module )
    ├── mrs_multimaster                            ( communication module )
    ├── mrs_rviz_plugins
    ├── mrs_serial
    ├── nimbro_network
    ├── rplidar_ros
    ├── terarangeron                               ( rangefinder driver )
    ├── tersus_gps_driver                          ( RTK GPS module )
    ├── trajectory_loader                          ( loader of trajectories )
    └── usb_cam                                    ( camera driver )
``` 


**bluefox2** is a camera driver for mBluefox camera.

**camera_base** is a dependency for bluefox2.

**mrs_gazebo_ros_interface** allows connecting objects in Gazebo simulator dynamically, e.g. to simulate grasping.

**mrs_multimaster** provides a way for sharing rostopics and call services between several individual ROS cores.

**tersus_gps_driver** is a driver for Tersus RTK GPS.

**terarangeron** is a driver for Teraranger rangefinder.

**trajectory_loader** allows loading desired trajectories for drones from CSV files.

**usb_cam** is a generic camera driver, which allows to rosify any /dev/video device.

TODO other
