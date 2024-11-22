---
title: PX4 Log Knowledgebase
---

### Vehicle flight mode

---

`vehicle_status/nav_state`:
```yaml
flight_modes_table = {\
0: ('Manual'), # red\
1: ('Altitude'), # yellow\
2: ('Position'), # green\
10: ('Acro'), # olive\
14: ('Offboard'), # light blue\
15: ('Stabilized'), # dark blue\
16: ('Rattitude'), # orange # all AUTO-modes use the same color\
3: ('Mission'), # purple\
4: ('Loiter'), # purple\
5: ('Return to Land'), # purple\
6: ('RC Recovery'), # purple\
7: ('Return to groundstation'), # purple\
8: ('Land (engine fail)'), # purple\
9: ('Land (GPS fail)'), # purple\
12: ('Descend'), # purple\
13: ('Terminate'), # purple\
17: ('Takeoff'), # purple\
18: ('Land'), # purple\
19: ('Follow Target'), # purple\
20: ('Precision Land'), # purple\
21: ('Orbit'), # purple\
```

### RC Inputs

---
`input_rc/values.00` : Roll\
`input_rc/values.01` : Throttle\
`input_rc/values.02` : Pitch\
`input_rc/values.03` : Yaw\
`input_rc/values.04` : Offboard\
`input_rc/values.05` : Flight Mode\
`manual_control_switches/*` : Switches\

### Log Profiles

---
In addition to the topics logged by the SD card, adding `Bit:4 - High rate` is enough to get values on actuators, rates, attitude, and sensors. Note that `Bit:3 - System identification` is a subset of topics included in `high_rate` topics and so it is not useful to enable them both. Although, enabling both is not harmful. In essence, unless flying on PX4's autonomy, `SDLOG_PROFILE:17` is recommended.
More details can be found in the PX4 file `src/modules/logger/logged_topics.cpp`.
