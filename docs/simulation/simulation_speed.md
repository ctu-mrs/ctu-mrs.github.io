---
layout: default
title: Run Simulation Faster than Real-time
parent: Simulation
nav_order: 3
---

# Run Simulation Faster than Real-time

Simulation In The Loop (SITL) can be run faster or slower than real-time.

The speed factor is set using the environment variable PX4_SIM_SPEED_FACTOR.
For example, to run the simulation at 2 times the real-time speed put the following command to your `session.yml` into `pre_window` part, or into `~/.bashrc`:

```bash
export PX4_SIM_SPEED_FACTOR=2
```

To run at half real-time:

```bash
export PX4_SIM_SPEED_FACTOR=0.5
```

### Example of setting `session.yml` with PX4_SIM_SPEED_FACTOR=2
```yml
name: simulation
root: ./
startup_window: status
pre_window: export UAV_NAME=uav1; export RUN_TYPE=simulation; export UAV_TYPE=t650; export WORLD_NAME=simulation; export PX4_SIM_SPEED_FACTOR=2
windows:
...
```

For more detail description see [px4 website](https://dev.px4.io/v1.10/en/simulation/#simulation_speed).
