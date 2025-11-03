---
title: Publishers
pagination_label: Publishers
description: ROS2 Publishers
---

# ROS2 Publishers

## Creating a latched publisher

Latching can be enabled through QoS.

1. Setup a QoS profile using a baseline of your choice.
```cpp
rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS();
```
2. Enable the `transient_local()` durability setting:
```cpp
qos_profile.transient_local();
```
1. Pass the QoS profile to the publisher handler using the `PublisherHandlerOptions`:
```cpp
mrs_lib::PublisherHandlerOptions ph_options;

ph_options.node = node_;
ph_options.qos = qos_profile;

ph_my_topic_ = mrs_lib::PublisherHandler<my_type>(ph_options, "~/topic_out");
```
