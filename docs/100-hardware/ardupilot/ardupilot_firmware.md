---
title: Ardupilot Firmware
---

# Flashing the Kakute-H7 firmware

The limited Kakute memory size doesn't allow to replace PX4 bootloader and firmware at the same time.
Therefore, first flash the [bootloader](https://firmware.ardupilot.org/Tools/Bootloaders/KakuteH7-bdshot_bl.bin) by executing
```bash
dfu-util -a 0 --dfuse-address 0x08000000 -D KakuteH7-bdshot_bl.bin
```
and then proceed to update the [firmware without bootloader](https://firmware.ardupilot.org/Copter/stable-4.6.3/KakuteH7-bdshot/arducopter.apj) through QGoundControl.
Only the version 4.6.3 has currently been tested.
