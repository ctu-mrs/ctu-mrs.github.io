---
layout: default
title: PX4 Configuration
parent: Hardware
---

## UNDER CONSTRUCTION
## UNDER CONSTRUCTION
## UNDER CONSTRUCTION
## UNDER CONSTRUCTION

# Pixhawk setup on a new drone
Follow this guide to setup a new drone with pixhawk for the MRS UAV system.

## HW setup
This guide is written for Pixhawk 4, because it is used on most of the MRS aerial platforms. But it is applicable for other versions of Pixhawk, with minor differences.

1. Pixhawk 4 comes with a power distribution/supply board. This board provides power distribution for motors (ESCs), connections for ESC signals, two redundant 5V power supplies for Pixhawk and current and voltage monitoring .
 
[![](fig/power_board.jpg "Pixhawk power board")](fig/power_board.jpg) | [![](fig/pixhawk4.jpg "Pixhawk 4")](fig/pixhawk4.jpg)

2. Install the power board into the frame, solder the motor connections and connect the ESC signal cables.
3. You can either solder the ESC signal cables to the M1-M8 ports, or connect them with standard servo cable connectors (servo cable solution is recommended by MRS).
4. Connect cables to both power outputs labeled as `PWR1` and `PWR2`.
5. Connect a cable for the ESC signals to the `IO-PWM-in` port (if you soldered the ESC signal cables) or the `FMU-PWM-in` port (if you used the servo connectors).

![](fig/PB_no_cables.jpg "Power board without cables") | ![](fig/PB_with_cables.jpg "Power board with cables attached")

6. Install Pixhawk into the drone frame and connect the power cables from the distribution board to the `POWER1` and `POWER2` slots.
7. Connect the ESC signal cable to the `I/O PWM OUT` port.
8. Connect the RC receiver. At MRS we use an OPTIMA SL reciver with SBUS output, so it is connected to the `DSM/SBUS RC` port.
9. Connect the onboard computer to the `TELEM2` port. This is a UART port which ensures communication between the Pixhawk and the onboard computer. We use an FTDI serial to USB converter to connect the Pixhawk with the onboard computer.
10. Optionally, you can connect other sensors, like a GPS module, rangefinder, etc. Note that the arming button and the buzzer are integrated into the GPS receiver, so if you want to use them either connect the GPS receiver or your own arming button and buzzer to the `GPS MODULE` port. The arming button has to be disabled in SW if it is not used (this is unsafe).

![](fig/Pixhawk_no_cables.jpg "Pixhawk without cables") | ![](fig/Pixhawk_with_cables.jpg "Pixhawk with cables attached")

# SD card setup
Pixawk has an SD card for flight logs and for additional configuration.
If you want to use the MRS system, you have to use our PX4 sd card configuration. You can find it in the [uav_core/miscellaneous](https://github.com/ctu-mrs/uav_core/tree/master/miscellaneous/pixhawk_sdcard_config) repository.
Create a folder called `etc` in the root of the SD card and copy the `extras.txt` file into this folder.
Install the SD card back into the Pixhawk.

#  SW setup
1. Download [QGroundControl](http://qgroundcontrol.com/)
2. Connect the Pixhawk to your computer with a USB cable and run QGroundControl
3. Update the Pixhawk firmware to the latest stable version. Optionally, you can use our [tweaked firmware](https://ctu-mrs.github.io/docs/hardware/px4_firmware.html) or your custom firmware.
4. Set the airframe type according to your drone (most common options are Generic Quadcopter, Generic Hexarotor and Genereic Octocopter)
5. Calibrate all the sensors according to the instructions in QGroundControl. Pixhawk has to be installed in the drone frame during calibration.

![](fig/Qground1.png "QGroundControl frame selection") | ![](fig/Qground2.png "QGroundControl sensor calibration")

6. Setup your radio. Pair your RC transmitter and your receiver and connect the receiver to Pixhawk.
7. Setup the channels on your Transmitter. Proper channel setup is important for some functionality of the MRS system. We use 8 RC channels:
  1. hadsf
  2. adsf
  3. adsf
  4. adf
  5.
8. You should see the RC channel values in QGroundControl. If you do not see the RC channels, check the connection of the receiver.
9. Calibrate your transmitter with the `Calibrate` button in QGroundControl and follow the instruction.

# ROS setup

## Additional Pixhawk configuration

# Garmin rangefinder throught Pixhawk

1. The Pixhawk's sd-card config should contain:
```
mavlink stream -d /dev/ttyS2 -s DISTANCE_SENSOR -r 100
```
2. Set parameter `SENS_EN_LL40LS` to i2c

More info: [https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html](https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html)

# Configuration

## Disabling/Enabling internal magnetometer

Disable: `EKF2_MAG_TYPE := None`

Enable: `EKF2_MAG_TYPE := Automatic`
