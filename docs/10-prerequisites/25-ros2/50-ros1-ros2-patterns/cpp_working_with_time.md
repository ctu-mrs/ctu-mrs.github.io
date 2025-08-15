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

Duration object storing an interval of `1 second`:
```cpp
rclcpp::Duration(std::chrono::duration<double>(1.0))
```

## rclcpp::Time

* creating the Time object out of a floating point time:
```cpp
rclcpp::Duration(std::chrono::duration<double>(1.0))
```

* incrementing time: you can add `rclcpp::Duration` to `rclcpp::Clock` , but it needs to have the same clock type:
```cpp
last_time_ + rclcpp::Duration(secs_passed, clock_->get_clock_type());
```
or
last_time_ + rclcpp::Duration(secs_passed, last_time_->get_clock_type());

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
