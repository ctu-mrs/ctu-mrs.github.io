---
layout: default
title: Buttons
parent: Introduction
---

# uav_core repository

[uav_core](uav_core) contains "the core" of our packages, literally.
Those are minimal required for the drone to fly.
Besides, it includes automated installation scripts and various miscellaneous additions, mainly for the hardware.

```

~/git/uav_core
   ├── installation
   │   ├── install.sh
   │   └── scripts 
   ├── lib
   ├── miscellaneous
   ├── README.md
   ├── ros_packages
   │   ├── mavros                        ( uav communication library )
   │   ├── mrs_controllers               ( feedback controlers )
   │   ├── mrs_general                   ( general config files and utilities )
   │   ├── mrs_lib                       ( libraries )
   │   ├── mrs_mavros_interface          ( mrs interface for mavros )
   │   ├── mrs_msgs                      ( ros message definitions for all packages )   
   │   ├── mrs_odometry                  ( senzor fusion and state estimation )
   │   ├── mrs_optic_flow                ( optic flow )
   │   ├── mrs_status                    ( terminal status viewer ) 
   │   ├── mrs_trackers                  ( control reference generator )
   │   ├── mrs_testing                   ( unit tests for the control pipeline )
   │   └── mrs_uav_manager               ( main control system )
   └── tmux_scripts

```

**mavros** TODO

**mrs_controllers** TODO

**mrs_general** TODO

**mrs_lib** TODO

**mrs_mavros_interface** TODO

**mrs_msgs** TODO

**mrs_odometry** TODO

**mrs_optic_flow** TODO

**mrs_status** TODO

**mrs_trackers** TODO

**mrs_testing** TODO

**mrs_mrs_uav_manager** TODO
