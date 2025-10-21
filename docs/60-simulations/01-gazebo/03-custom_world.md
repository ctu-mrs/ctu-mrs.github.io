---
title: Custom simulation world
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# How to start the simulation with a custom world

![](fig/custom_world.jpg)

To [start the simulation](./01-howto.md), we use ROS launch files

```bash
ros2 launch mrs_uav_gazebo_simulator simulation.launch.py
```

The launch [`simulation.launch.py`](https://github.com/ctu-mrs/mrs_uav_gazebo_simulator/blob/ros2/launch/simulation.launch.py) file contains this argument

```python
declare_world_file_arg = DeclareLaunchArgument(
    'world_file',
    default_value = PathJoinSubstitution([
            pkg_mrs_common_gazebo_resources, 'worlds', 'grass_plane.sdf'
        ]),
    description='Path to the SDF world file'
)
```

specifying the default world to be `grass_plane.sdf` from the [`mrs_gazebo_common_resources`](https://github.com/ctu-mrs/mrs_gazebo_common_resources/tree/ros2) package.

## Load a custom world

### Load world from `mrs_gazebo_common_resources`

Pass the **world_file** (e.g., `forest.sdf`) as an argument to the launch file

```bash
worlds="$(ros2 pkg prefix --share mrs_gazebo_common_resources)/worlds"
ros2 launch mrs_uav_gazebo_simulator simulation.launch.py world_file:="$worlds/forest.sdf"
```

### Load arbitrary custom world

Pass the **world file** as an argument to the launch file using `find` to locate your package

```bash
ros2 launch mrs_uav_gazebo_simulator simulation.launch.py world_file:="$(find custom_gazebo_resources)/worlds/custom_world.sdf"
```

or use the absolute path of the world file

```bash
ros2 launch mrs_simulation simulation.launch.py world_file:=/path/to/world/custom_world.sdf
```

## How to create a custom world

### Create the world manually in a text editor

Create a [completely new](https://gazebosim.org/docs/latest/sdf_worlds/#sdf-worlds) *.sdf* file or copy & modify an existing file (e.g., [forest.sdf](https://github.com/ctu-mrs/mrs_gazebo_common_resources/blob/ros2/worlds/forest.sdf)).
Make sure your world file contains line
```xml
    <plugin name="mrs_gazebo_static_transform_republisher_plugin" filename="libMrsGazeboCommonResources_StaticTransformRepublish.so"/>
```
The included [mrs_gazebo_static_transform_republisher](https://github.com/ctu-mrs/mrs_gazebo_common_resources/blob/master/src/world_plugins/static_transform_republisher.cpp) plugin provides static [transforms](https://ctu-mrs.github.io/docs/system/frames_of_reference.html) of all the [spawned sensors](https://ctu-mrs.github.io/docs/simulation/howto.html#2-spawn-a-drone-drones) to the `<uav_name>/fcu` frame.

### Create the world using Gazebo

1. Start a Gazebo simulation, `gz sim`
2. Insert models to the world as you wish using the Gazebo GUI
3. Save world to file: `File -> Save as`
4. Make sure the created file contains the `mrs_gazebo_static_transform_republisher` plugin.

## Common issues

### No static transformations

`rosrun rqt_tf_tree rqt_tf_tree` shows no transformations between spawned sensors and the `<uav_name>/fcu` frame?
Make sure your world file contains the `mrs_gazebo_static_transform_republisher` plugin as described [above](https://ctu-mrs.github.io/docs/simulation/custom_world.html#create-the-world-manually-in-a-text-editor).
Lack of static transformations might also prevent start of some systems and hence prevent taking off.

## Further reading

https://gazebosim.org/docs/latest/ros2_launch_gazebo/

http://gazebosim.org/docs/latest/spawn_urdf/
