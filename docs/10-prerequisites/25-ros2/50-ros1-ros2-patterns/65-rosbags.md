---
title: ROS2 Bags
pagination_label: ROS2 Bags
description: ROS2 Bags
---

# ROS2 Bags

Ros2 `mcap` bags are robust and efficient.
However, if you cut the recording process abruptly, the bag is likely to end up corrupted.
No worries, your data should be fine.
To fix your bag, first try `ros2 bag reindex`.
In some cases, however, it is not enough and you need to install [`mcap`](https://mcap.dev/) toolkit, allowing the `mcap recover` command.

