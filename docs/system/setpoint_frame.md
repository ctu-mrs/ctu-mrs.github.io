---
layout: default
title: Coordinate frames of the command setpoint
parent: The UAV system
nav_order: 6
---

# What to keep an eye out for when setting a command setpoint for the MRS control pipeline

## Problem description

Consider the following typical situation: You are attempting to make the UAV remain stationary at its current position.
An intuitive (yet wrong) approach is to subscribe to the UAV odometry and publish the current UAV position as the desired command setpoint for the control pipeline.
However, this will most probably cause your UAV to drift, as is illustrated in the following figure.
This is due to the fact, that the UAV is in practice *never* precisely at the commanded position, even when stationary (because of various disturbances, delays, noise in the odometry, UAV dynamics etc.).

More specifically, when you command the UAV at time $t_1$ to move to its current measured position $\mathbf{x}_c\left(t_1\right) = \mathbf{x}_m\left(t_1\right)$, it will attempt to go there as well as it can.
However, at time $t_2$, it will actually reach some other (ideally very close) position $\mathbf{x}_m\left(t_2\right) \neq \mathbf{x}_c\left(t_1\right)$.
Now again, you command the UAV again to go to its measured position $\mathbf{x}_c\left(t_2\right) = \mathbf{x}_m\left(t_1\right)$.
You can already see, that the commanded position at $t_2$ is different, than the on at $t_1$.
Even though you wanted your UAV to stay put, you are actually commanding it to move!

![uav_drift](fig/setpoint_frame_fig.png)

Unfortunately, what you have effectively created is a positive feedback causing the UAV to drift in a certain direction, which is given by random disturbances in the system.
