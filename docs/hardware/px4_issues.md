---
layout: default
title: PX4 issues
parent: Hardware
---

# Discovered issues of PX4 firmware  

## The drone falls down after aggressive maneuvers

### The issue:

Newer DJI ESCs introduced the active braking feature that supposedly improves the dynamics of the UAV. The ESCs actively brake the motors when minimal-width PWM pulse is received. The PX4 controller does not cope well with the braked motors, so the drone tips over and falls down. This happens mostly during aggressive changes of roll or pitch.

### Affects: 

* so far observed with DJI 430 Lite ESCs + Pixhawk mini or Pixhawk 2.1
* Pixhawk 2.1 is mostly able to recover, but still does a weird spiral movement and loses altitude
* manual, altitude, position modes are all affected
* autonomous flights are mostly smooth thus unaffected, but the manual takeover by safety pilots often involves aggressive tilting that triggers active braking

### To reproduce:

The easiest way to reproduce the issue is to fly in position hold mode, move the roll or pitch stick to its extreme position and then let it quickly slide back to zero.
Be sure to remove all sensors and expensive parts from the drone and fly in ~2m altitude to minimize the damage from the crash. The drone usually falls on its top.

### Solution:

Increase the minimal PWM width (around 1130us did the trick for Pixhawk 2.1 with DJI 430 Lite ESCs)

**WARNING!!** The motors will start spinning slowly after arming with the RC similarly to DJI Phantom etc. At least we won't forget to disarm after landing anymore.
