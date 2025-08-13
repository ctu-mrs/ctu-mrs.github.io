---
title: C++ components (nodelets)
pagination_label: C++ components (nodelets)
description: C++ components (nodelets)
---

# `onInit()`

There is no onInit() method like in ROS1.
Now, you need to implement the constructor of `rclcpp::Node()`.

However, you cannot obtain a pointer to the node in the constructor (while it is still running).
Therefore, we create our own `initialize()` method as a one-shot timer that contain most of the init routines.
We can use the constructor solely for starting the timer.
