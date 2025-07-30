---
title: ROS2 CmakeLists
pagination_label: ROS2 CmakeLists
description: ROS2 CmakeLists
---

# ROS2 CmakeLists

## `include_directories()`

In ROS2 you have to explicitly list all the packages you are including from:
```
...
include_directories(
  ...
  ${mrs_lib_INCLUDE_DIRS}
  ${<some_other_package>_INCLUDE_DIRS}
}
```

## exporting nodeletes (components)

Components are declared in CmakeLists, instead of package.xml (like it was in ROS1):
```
rclcpp_components_register_nodes(MrsMultirotorSimulator_Simulator PLUGIN "mrs_multirotor_simulator::MultirotorSimulator" EXECUTABLE MrsMultirotorSimulator_Simulator)
```

## exporting pluginlib plugins

You need to provide path to the `.xml` file with plugin description in CmakeLists, instaed of in `package.xml` like it was in ROS1:
```
# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)
```
