---
layout: default
title: PX4 Dshot setup
parent: Hardware
---

# Dshot ESC communication
By default, ESCs and Pixhawk communicate via PWM.
This analog approach has a few disadvantages, it has relatively high latency and it requires calibration.
Pixhawk and the ESCs have to "agree" which pulse width corresponds to what levels of throttle.
Sometimes, ESCs can enter the calibration sequence by accident, this can happen when you use an anti-spark connector on you battery, or you have some intermittent contacts when pluging the battery in.
The result is bad PWM calibration, which can lead to unexpected motor behaviour.
To fix all these problems, we can use Dshot - a digital communication protocol.
Dshot has lower latency, and no calibration, since it is digital.

## How to setup Dshot
1. Make sure that your ESCs support Dshot.
2. Using QGroundControl, modify these parameters:
  * `DSHOT_CONFIG` to one of the values (`150, 300, 600, 1200`). These values corresponds to communication speed in kb/s. Higher value means lower latency. Not all ESCs support all communication speeds.
  * `SYS_USE_IO` to `0`
3. Connect the motor signals to the `FMU PWM OUT` port. By default, they are connected to the `I/O PWM OUT` port.

That is it, you can now try to arm your drone and spin the motors (without props).
If they spin, you are good to go.
You can also modify the `DSHOT_MIN` parameter, it sets the minimum possible throttle.
Make sure that the propellers always spin when you arm the drone.

In the past version of PX4 firmware, enabling Dshot and setting `SYS_USE_IO` to `0` disabled the functionality of the safety arming button. As of writing this guide, this issue is no longer present (PX4 version 1.11.2).

