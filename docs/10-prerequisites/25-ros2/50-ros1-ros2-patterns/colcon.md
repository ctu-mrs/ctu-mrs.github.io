---
title: Colcon configuration
pagination_label: ROS2 workspace colcon configuration 
description: ROS2 colcon configuration
---

# Colcon configuration

Colcon can be configed in two ways.

1. Command line arguements
```bash
colcon build --symlink-install --continue-on-error --executor parallel --parallel-workers 5
```

2. `colcon_defaults.yaml` file in the root of your workspace.
Running a simple `colcon build` will automatically use these options.
```yaml
build:
  symlink-install: True
  continue-on-error: True
  executor: parallel
  parallel-workers: 5
```
