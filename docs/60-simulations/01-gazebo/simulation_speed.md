---
title: Run Simulation Faster than Real-time
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

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

# Run Simulation SLOWER than Real-time

Running simulation slower using the `PX4_SIM_SPEED_FACTOR` variable causes problems when simulating multiple UAVs.
To slow down the simulation instead change the max real-time update rate of Gazebo some time after the start of the simulation, i.e., include this in your `session.yml` file to slow down the simulation to 0.5 real-time factor:
```yml
  - gz_rate:
      layout: tiled
      panes:
        - waitForOdometry; sleep 5; gz physics -u 125
```
`gz physics -u 250` corresponds to real-time factor `1.0`, reduce the update rate accordingly to reach your desired real-time factor.
