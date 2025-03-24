---
title: ROS2 logging
pagination_label: ROS2 logging
description: ROS2 logging
---

# ROS2 logging

## Standard logging

In ROS1
```cpp
ROS_INFO("dog");
```

in ROS2
```cpp
RCLCPP_INFO(node_->get_logger(), "dog");
```

## Throttled logging

In ROS1
```cpp
ROS_INFO_THROTTLED(1.0, "dog");
```

In ROS2
```cpp
RCLCPP_INFO_THROTTLED(node_->get_logger(), *(node_->get_clock()), 1000, "dog");
```
the `delta` time between messages is given in milliseconds.
