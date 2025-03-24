---
title: workspace profiles
pagination_label: ROS2 workspace profiles
description: ROS2 workspace profiles
---

# ROS2 workspace profiles

## Setting work-space wide configurations

Create a file `colcon_defaults.yaml` in the workspace's root:

```yaml
build:
  executor: sequential
  parallel-workers: 8
```

## Mixin

**"Manually adding arguments but with extra steps"**

Add the official mixin repository:
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
```

Update the mixins:
```bash
colcon mixin update
```

Show the mixins:
```bash
colcon mixin show
```

Build with particular mixin:
```bash
colcon build --mixin rel-with-deb-info
```

Use mixins automatically: Use the `colcon_defaults.yaml` file:

```yaml
build:
  mixin:
    - debug
```
