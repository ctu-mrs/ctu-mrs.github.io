---
layout: default
title: Run Simulation Faster than Real-time
parent: Simulation
---

# Run Simulation Faster than Real-time

Simulation In The Loop (SITL) can be run faster or slower than real-time.

The speed factor is set using the environment variable PX4_SIM_SPEED_FACTOR.
For example, to run the simulation at 2 times the real-time speed put the following line into `~/.bashrc`:

```bash
export PX4_SIM_SPEED_FACTOR=2
```

To run at half real-time:

```bash
export PX4_SIM_SPEED_FACTOR=0.5
```

For more detail description see [px4 website](https://dev.px4.io/v1.10/en/simulation/#simulation_speed).
