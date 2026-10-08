---
title: Ardupilot Kakute-H7 Configuration
---

# Ardupilot setup on Kakute-H7 drone (most likely a Robofly)

Follow this guide to setup a new drone with the Ardupilot autopilot for the ROS2 version of the MRS UAV system.
As of the moment, you have to build the [MRS Ardupilot API package](git@mrs.fel.cvut.cz:nekovfra/mrs_uav_ardupilot_api.git).

## HW setup

TODO: HW guys and their magic

## SD card setup

Unlike the PX4, ardupilot SD card setup requires the serial stream configuration files ('message-intervals-chan0.txt' and 'message-intervals-chan1.txt' from 'mrs_uav_ardu
/real_hw_setup_material) to be placed in the card's root directory.
The Ardupilot convention is that the numbering of the '.txt' file reflects the numbering of serial ports configured for Mavlink protocol.
In our case for example, the serial0, which we access using the QGroundControl, will be configured with 'message-intervals-chan0.txt' file, and serial4 with 'message-intervals-chan1.txt' file, as the serial1-3 are not configured for Mavlink.

## SW setup

TBD
