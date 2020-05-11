---
layout: default
title: PX4 Firmware
parent: Hardware
---

# Flashing our custom firmware

## Getting the firmware

If you have our [simulation](https://github.com/ctu-mrs/simulation), you already have the firmware cloned under `simulation/ros_packages/px4` and the build dependencies satisfied.

If you don't have it, just clone as
```bash
git clone https://github.com/ctu-mrs/px4_firmware.git
```
and install its dependencies according to the [manual](https://dev.px4.io/master/en/setup/building_px4.html).

## Compiling the firmware

1. go into the root of the firmware repository
2. `make <target>`, where the <target> depends on the version of the hardware:
  * Pixhawk 4: make px4_fmu-v5_default
  * Pixracer: make px4_fmu-v4_default
  * Pixhawk 3 Pro: make px4_fmu-v4pro_default
  * Pixhawk Mini: make px4_fmu-v3_default
  * Pixhawk 2: make px4_fmu-v3_default
  * mRo Pixhawk: make px4_fmu-v3_default (supports 2MB Flash)
  * HKPilot32: make px4_fmu-v2_default
  * Pixfalcon: make px4_fmu-v2_default
  * Dropix: make px4_fmu-v2_default
  * MindPX/MindRacer: make airmind_mindpx-v2_default
  * mRo X-2.1: make auav_x21_default
  * Crazyflie 2.0: make bitcraze_crazyflie_default
  * Intel® Aero Ready to Fly Drone: make intel_aerofc-v1_default
  * Pixhawk 1: make px4_fmu-v2_default

## Uploading the firmware

The firmware file with the extension `.px4` can be found in the build subfolder.
Upload it using the [Qgroundcontrol](http://qgroundcontrol.com/) utility.

# Garmin rangefinder throught Pixhawk

1. The Pixhawk's sd-card config should contain:
```
mavlink stream -d /dev/ttyS2 -s DISTANCE_SENSOR -r 100
```
2. Set parameter `SENS_EN_LL40LS` to i2c

More info: [https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html](https://dev.px4.io/master/en/middleware/modules_driver_distance_sensor.html)
