---
layout: default
title: ROS remote
parent: Software
---

# Connecting to ROS core remotely

ROS allows multiple computers to communicate and share topics and services via a local network.
This feature is useful for remote testing and simulations, where different ROS nodes can run on multiple machines as if they were on a single computer.

Configure the client's computers to enable this feature.
Add following two lines to **.bashrc** on the client computer(s)

```bash
export ROS_IP=<machine-ip-addr>
export ROS_MASTER_URI=http://<server-hostname>:11311
```

The `<server-hostname>` should be specified in `/etc/hosts` on clients computers and `<client-hostname>` should be specified on the server computer.

When you are done, don't forget to comment or delete the configuration on the client.
Otherwise, ROS will not be able to start.

For more information, follow to [wiki.ros.org/ROS/Tutorials/MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).
