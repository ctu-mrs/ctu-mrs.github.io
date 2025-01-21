---
title: Remote ROS master
pagination_label: Remote access to ROS core
description: Remote access to ROS core
---

# Connecting to ROS on different machine (a robot)

ROS allows multiple computers to communicate and share topics and services via a local network.
This feature is useful for remote testing and simulations, where different ROS nodes can run on multiple machines as if they were on a single computer.

**To connect two example machines**

1) **Server (robot) with IP `192.168.68.101` and hostname `uav`**
2) **Client (notebook) with IP `192.168.68.5` and hostname `notebook`**

**you need to setup both the machines followingly.**

## Server computer (the robot)

Add following lines to `~/.bashrc`:
```bash
export ROS_MASTER_URI=http://localhost:11311
```

Add following lines to the `/etc/hosts`:
```bash
192.168.68.5 notebook
```

**Important** (skipping these steps may kill ROS on the robot once it disconnects from Wifi):

- Do **NOT** export `ROS_IP` to `~/.bashrc`.
- Remove the server's (the robot's) own hostname in `/etc/hosts` except of `127.0.1.1`.
  More than one instance of `192.168.68.101` in `/etc/hosts` is not allowed.
  The top of the `/etc/hosts` file of the robot should look like:
```bash
127.0.0.1 localhost
127.0.1.1 uav

192.168.68.101 uav
192.168.68.5   notebook
```

## Client computer (the notebook)

Add following lines to `~/.bashrc`:
```bash
export ROS_MASTER_URI=http://192.168.68.101:11311
```

Add following lines to `/etc/hosts`:
```bash
192.168.68.101 uav
```

## Troubleshooting

- Both IPs must be in the same subnet.
- Once you set up accordingly, you should be able to mutually ping the machines using only their hostnames (i.e., `ping uav` from `notebook` and vice versa).
- The hostnames must be **exact**. If the server has hostname `uav`, the client **must** specify `uav` and nothing else (e.g., `uav1`). In case it mismatches, `ping` will work between the machines but ROS topic transfer will NOT work.
- There can be multiple clients/servers in `/etc/hosts`, but the IPs must be unique.
- To be able to run ROS on the client again, set `ROS_MASTER_URI` back to `export ROS_MASTER_URI=http://localhost:11311`.
- For more information, follow to [wiki.ros.org/ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).
