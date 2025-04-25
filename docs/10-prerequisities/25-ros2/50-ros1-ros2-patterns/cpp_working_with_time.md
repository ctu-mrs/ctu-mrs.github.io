---
title: ROS Time
pagination_label: Working with ROS Time in ROS2
description: Working with ROS Time in ROS2
---

# ROS Time in ROS2

## The clock object

## Comparing times

## rclcpp::Rate

## rclcpp::Duration

Duration object storing an interal of `1 second`:
```cpp
rclcpp::Duration(std::chrono::duration<double>(1.0))
```

## rclcpp::Time

* creating the Time object out of a floating point time:
```cpp
rclcpp::Duration(std::chrono::duration<double>(1.0))
```

## Sleeping

What was in ROS1 done as:
```cpp
ros::Duration(duration).sleep();
```
is in ROS2 done as:
```cpp
clock_->sleep_for(std::chrono::duration<double>(duration));
```

## Simulation time
