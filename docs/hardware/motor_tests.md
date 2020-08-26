---
layout: default
title: Motor tests
parent: Hardware
---

# Motor tests
All motor tests are done with a Turnigy MultiStar BLheli_32 51A ESC, with the 32.7 BLheli firmware. ESC config can be found here - https://github.com/ctu-mrs/uav_core/blob/master/miscellaneous/blheli32_esc_config/T650/T650_M1.ini
Current and voltage measurements are done by the ESC and thrust is measured with a 5 kg load cell, coupled with HX711 amplifier.
Tests are done with fully charged lithium batteries, as we do not have a powerful enough adjustable power supply.

## Plastic 9545 self-tightening propeller

{: .fw-500 }
| Throttle (%) | Thrust (g) | RPM   | Voltage (V) | Current (A) | Power (w) | Efficiency (g/W) |
| :---:        | :---:      | :---: | :---:       | :---:       | :---:     | :---:            |
| 50           | 401        | 5688  | 16.66       | 3.09        | 51.48     | 7.79             |
| 55           | 464        | 6154  | 16.64       | 3.92        | 65.23     | 7.11             |
| 60           | 529        | 6508  | 16.61       | 4.80        | 79.73     | 6.64             |
| 65           | 596        | 6911  | 16.58       | 5.73        | 95.00     | 6.27             |
| 70           | 656        | 7213  | 16.54       | 6.72        | 111.15    | 5.90             |
| 75           | 713        | 7665  | 16.51       | 7.78        | 128.45    | 5.55             |
| 80           | 787        | 7788  | 16.48       | 9.05        | 149.14    | 5.28             |
| 85           | 855        | 8122  | 16.44       | 10.24       | 168.35    | 5.08             |
| 90           | 911        | 8608  | 16.40       | 11.48       | 188.27    | 4.84             |
| 95           | 971        | 8870  | 16.36       | 12.81       | 209.57    | 4.63             |
| 100          | 1024       | 9068  | 16.32       | 14.26       | 232.72    | 4.40             |
{: .fw-700 }

