---
layout: default
title: PX4 Configuration
parent: Hardware
---

# Pixhawk setup on a new drone
Follow this guide to setup a new drone with the Pixhawk autopilot for the MRS UAV system.

## HW setup
This guide is written for Pixhawk 4, because it is used on most of the MRS aerial platforms. But it is applicable for other versions of the Pixhawk, with minor differences.

- The Pixhawk 4 comes with a power distribution/supply board. This board provides power distribution for motors (ESCs), connections for ESC signals, two redundant 5V power supplies for Pixhawk and current and voltage monitoring .

[![](fig/power_board.jpg "Pixhawk power board")](fig/power_board.jpg) | [![](fig/pixhawk4.jpg "Pixhawk 4")](fig/pixhawk4.jpg)

- Install the power board into the frame, solder the motor connections and connect the ESC signal cables.
- You can either solder the ESC signal cables to the M1-M8 ports, or connect them with standard servo cable connectors (servo cable solution is recommended by MRS).
- Connect cables to both power outputs labeled as `PWR1` and `PWR2`.
- Connect a cable for the ESC signals to the `IO-PWM-in` port (if you soldered the ESC signal cables) or the `FMU-PWM-in` port (if you used the servo connectors).

[![](fig/PB_no_cables.jpg "Power board without cables")](fig/PB_no_cables.jpg) | [![](fig/PB_with_cables.jpg "Power board with cables attached")](fig/PB_with_cables.jpg)

- Install the Pixhawk into the drone frame and connect the power cables from the distribution board to the `POWER1` and `POWER2` slots.
- Connect the ESC signal cable to the `I/O PWM OUT` port.
- Connect the RC receiver. At MRS we use an OPTIMA SL reciver with SBUS output, so it is connected to the `DSM/SBUS RC` port.
- Connect the onboard computer to the `TELEM2` port. This is a UART port which ensures communication between the Pixhawk and the onboard computer. We use an FTDI serial to USB converter to connect the Pixhawk with the onboard computer.
-. Optionally, you can connect other sensors, like a GPS module, rangefinder, etc. Note that the arming button and the buzzer are integrated into the GPS receiver, so if you want to use them either connect the GPS receiver or your own arming button and buzzer to the `GPS MODULE` port. The arming button has to be disabled in SW if it is not used (this is unsafe).

[![](fig/Pixhawk_no_cables.jpg "Pixhawk without cables")](fig/Pixhawk_no_cables.jpg) | [![](fig/Pixhawk_with_cables.jpg "Pixhawk with cables attached")](fig/Pixhawk_with_cables.jpg)

## SD card setup
Pixawk has an SD card for flight logs and for additional configuration.
If you want to use the MRS system, you have to use our PX4 sd card configuration. You can find it in the [uav_core/miscellaneous](https://github.com/ctu-mrs/uav_core/tree/master/miscellaneous/pixhawk_sdcard_config) repository.
Create a folder called `etc` in the root of the SD card and copy the `extras.txt` file into this folder.
Install the SD card back into the Pixhawk.

##  SW setup
- Download [QGroundControl](http://qgroundcontrol.com/)
- Connect the Pixhawk to your computer with a USB cable and run QGroundControl
- Update the Pixhawk firmware to the latest stable version. Optionally, you can use our [tweaked firmware](https://ctu-mrs.github.io/docs/hardware/px4_firmware.html) or your custom firmware.
- Set the airframe type according to your drone (most common options are Generic Quadcopter, Generic Hexarotor and Genereic Octocopter)
- Calibrate all the sensors according to the instructions in QGroundControl. Pixhawk has to be installed in the drone frame during calibration.

[![](fig/Qground1.png "QGroundControl frame selection")](fig/Qground1.png) | [![](fig/Qground2.png "QGroundControl sensor calibration")](fig/Qground2.png)

- Setup your radio. Pair your RC transmitter and your receiver and connect the receiver to the Pixhawk. You should see the RC channel values in QGroundControl. If you do not see the RC channels, check the connection of the receiver.
- Setup the channels on your Transmitter. Proper channel setup is critical for the functionality of the MRS UAV system. 8 RC channels are used:

| Channel     | Function             | Description                                                                                                |
| :---------- | :----------          | :------                                                                                                    |
| 1           | Roll                 | manual control input                                                                                       |
| 2           | Throttle             | manual control input                                                                                       |
| 3           | Pitch                | manual control input                                                                                       |
| 4           | Yaw                  | manual control input                                                                                       |
| 5           | Offboard switch      | Used to switch the drone in and out of Offboard mode                                                       |
| 6           | Flight mode switch   | Changes the PX4 flight mode when you fly manually                                                          |
| 7           | MRS system switch    | Switch used to trigger some functionality of the MRS UAV system, like the "remote" mode                    |
| 8           | MRS emergency switch | Triggers an emergency behaviour (e-hover, e-land, failsafe land) according to MRS UAV system configuration |

- The following switches are used by default on the RC transmitters:

[![](fig/RC1.jpg "RC switch configuration")](fig/RC1.jpg) | [![](fig/RC2.jpg "RC switch configuration")](fig/RC2.jpg)

- If you need to guickly remember what each switch is designated to, you can find it in the RC settings. The MRS team mostly uses Radiomaster TX16s RCs with OpenTX and as you can see from the following screenshot, the switches used are `SB`,`SC`,`SD` and `SH` respectively.

[![](fig/rc_3_mixes.bmp "RC Mixes screenshot")](fig/rc_3_mixes.bmp)

- Calibrate your transmitter with the `Calibrate` button in QGroundControl and follow the instruction.
- Setup the flight modes. Select Channel 5 as Offboard switch channel and channel 6 as Position Control switch channel and Mode Channel. Set Flight Mode 1 as Manual, Flight Mode 4 as Altitude and Flight Mode 6 as position (3 position switch is used at channel 6 to switch between those three modes).


[![](fig/Qground3.png "QGroundControl radio setup")](fig/Qground3.png) | [![](fig/Qground4.png "QGroundControl flight modes setup")](fig/Qground4.png)

- Calibrate your ESCs in the `Power` section. You can also configure your battery here (not needed for the MRS UAV system). Note that the `Power` icon may stay red, you can ignore this.
- In the `Safety` section, configure failsafe actions. Standard MRS configuration is Warning for Low Battery Failsafe Trigger, Land mode for RC Loss Failsafe Trigger and Land imediately in the Return To Launch Settings. No other triggers are activated (Object detection, data link loss etc.).
- Setup the RC loss failsafe. This failsafe is activated when the drone is flying manually (not in offboard mode) and the RC signal is lost. The RC receiver on the drone is configured to output abnormally low throttle signal when RC is lost, which is detected by the Pixhawk. This guide is for the Hitec Optima receivers, if you are using a different receiver the configuration steps may be different. To configure RC loss failsafe, follow these steps:
  * Turn on your RC transmitter and receiver.
  * Push you RC transmitter's throttle stick to the lowest level, then trim the throttle channel all the way down and use sub-trims to trim it even lower (we want to achieve the lowest possible value at the throttle CH2 channel).
  * Do not move the other sticks, leave them in centered positions.
  * Press the button on the RC receiver until the red LED turns off. Then release the button. The red and blue LEDs will start flashing for a while. This saves the current RC configuration as the output which the RC receiver produces when RC signal is lost.
  * In QGroundControl go to the `Parameters` section and set parameter `RC_MAP_FAILSAFE` to `Channel 2` and `RC_FAILS_THR` to `950 us`.
  * Restart the Pixhawk.
  * Now when you turn off your RC transmitter, QGroundControl should report `manual control lost` and when you turn your RC back on, it should report `manual control regained`.

# ROS setup

Once Pixhawk is connected to the computer via the serial to USB, it will show up as `/dev/ttyUSB0` (or something similar).
Run this command: (replace `/dev/ttuUSB0` with your device path)

```bash
udevadm info -p  $(udevadm info -q path -n /dev/ttyUSB0) | grep 'SERIAL_SHORT\|VENDOR_ID\|MODEL_ID'
```

You should get something like this:

```bash
E: ID_MODEL_ID=6001
E: ID_SERIAL_SHORT=A50285BI
E: ID_VENDOR_ID=0403
```

Create a new file in `/etc/udev/rules.d/` (you will need sudo privileges) and call it `99-usb-serial.rules`. Paste this line into the file:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A50285BI", SYMLINK+="pixhawk",OWNER="mrs",MODE="0666"
```

Replace idVendor, idProduct and serial with you values, and change the OWNER name to your user name. Now, if you disconnect and reconnect the Pixhawk, it should show up as `/dev/pixhawk`. Now you should be able to run mavros:

```bash
roslaunch mrs_uav_general mavros_uav.launch
```

# Garmin rangefinder throught Pixhawk

- The Pixhawk's sd-card config should contain:
```
mavlink stream -d /dev/ttyS2 -s DISTANCE_SENSOR -r 100
```
- Set parameter `SENS_EN_LL40LS` to i2c

More info: [https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html](https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html)

# Configuration

## Disabling/Enabling internal magnetometer

Disable: `EKF2_MAG_TYPE := None` and `SYS_HAS_MAG := 0`

Enable: `EKF2_MAG_TYPE := Automatic` and `SYS_HAS_MAG := 1`
