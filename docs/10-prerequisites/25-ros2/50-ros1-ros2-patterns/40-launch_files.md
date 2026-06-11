---
title: Launch files
pagination_label: ROS2 Launch files
description: ROS2 Launch files
---

# Launch Configuration

ROS2 provides the option to write your launch configuration files using `python` as well as `xml`.
Since the python files are more verbose and future proof, it is better to stick with the `python` API.
Below are some of the launch arguements that must be used with all the nodes.

## Standalone/Shared

While standard ROS 2 nodes can run as independent OS processes, modern ROS 2 design heavily relies on Component Composition, where nodes are compiled as shared libraries and loaded into an executing process called a Component Container.
All the nodes running within a container share resources such as available threads (in case of a multi-threaded container) and have zero-copy operations while communicating with each other.
The following pattern can be used to load a node into an existing container or run it inside a new one based on the conditional arguement inside the `ComposableNodeContainer`.
An example of the launch configuration can be seen at [uav_manager.launch.py](https://github.com/ctu-mrs/mrs_uav_managers/blob/219ff904250c8cf5d366a29ffe33f4021635e9c9/launch/uav_manager.launch.py#L43)

```
# #{ standalone

standalone = LaunchConfiguration("standalone")

declare_standalone = DeclareLaunchArgument(
    "standalone",
    default_value="true",
    description="Whether to start a as a standalone or load into an existing container.",
)

ld.add_action(declare_standalone)

# #} end of standalone

# #{ shared_container_name

shared_container_name = LaunchConfiguration("shared_container_name")

declare_shared_container_name = DeclareLaunchArgument(
    "shared_container_name",
    default_value="",
    description="Name of an existing container to load into (if standalone is false)",
)

ld.add_action(declare_shared_container_name)

# #} end of shared_container_name

# #{ default_node

default_node = ComposableNode(

    package=pkg_name,
    plugin=pkg_name+'::PluginName',
    namespace='',
    name=namespace,

    parameters=[
        #params
    ],

    remappings=[
        # subscribers
    ],
)

load_into_existing = LoadComposableNodes(
    target_container=shared_container_name,
    composable_node_descriptions=[default_node],
    condition=UnlessCondition(standalone)
)

ld.add_action(load_into_existing)

# #} end of default_node

# #{ standalone container

standalone_container = ComposableNodeContainer(
    namespace=uav_name,
    name=namespace+'_container',
    package='rclcpp_components',
    executable='component_container_mt',
    output="screen",
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[default_node],
    condition=IfCondition(standalone)
)

ld.add_action(standalone_container)
# #} end of standalone container
```

## Time

Currently, each and every ROS node needs to have a parameter `use_sim_time` in order to use the `/clock` topic for time synchronization.
If this parameter is missing, the behaviour may be undefined.
An example of the launch configuration can be seen at [uav_manager.launch.py](https://github.com/ctu-mrs/mrs_uav_managers/blob/219ff904250c8cf5d366a29ffe33f4021635e9c9/launch/uav_manager.launch.py#L146)

```
# #{ use_sim_time

use_sim_time = LaunchConfiguration("use_sim_time")

ld.add_action(
    DeclareLaunchArgument(
        "use_sim_time",
        default_value=os.getenv("USE_SIM_TIME", "false"),
        description="Should the node subscribe to sim time?",
    )
)

# #} end of use_sim_time

# Pass the parameter to the node
default_node = ComposableNode(
    package=pkg_name,
    plugin=pkg_name + "::PluginName",
    namespace="",
    name=namespace,
    parameters=[
        {"use_sim_time": use_sim_time},
    ],
    remappings=[
        # subscribers
    ],
)
```

## Logging

The level of logging (INFO, DEBUG, FATAL) can also be configured within the launch file as a launch arguement.
An example of the launch configuration can be seen at [uav_manager.launch.py](https://github.com/ctu-mrs/mrs_uav_managers/blob/219ff904250c8cf5d366a29ffe33f4021635e9c9/launch/uav_manager.launch.py#L170)

```
ld.add_action(DeclareLaunchArgument(name="log_level", default_value="info"))
standalone_container = ComposableNodeContainer(
...
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
...
)
```

## Custom Configs

The lanuch files can use a `custom_config` launch arguement which can be used to supply customised parameters that overwrite the default parameters loaded by the node.
Add the following lines before declaring a `ComposableNode` object.
An example of the launch configuration can be seen at [uav_manager.launch.py](https://github.com/ctu-mrs/mrs_uav_managers/blob/219ff904250c8cf5d366a29ffe33f4021635e9c9/launch/uav_manager.launch.py#L71)

```
custom_config = LaunchConfiguration("custom_config")

# this adds the args to the list of args available for this launch files
# these args can be listed at runtime using -s flag
# default_value is required to if the arg is supposed to be optional at launch time
ld.add_action(
    DeclareLaunchArgument(
        name="custom_config",
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    )
)

# behaviour:
# custom_config == "" => custom_config: ""
# custom_config == "/<path>" => custom_config: "/<path>"
# custom_config == "<path>" => custom_config: "$(pwd)/<path>"
custom_config = IfElseSubstitution(
    condition=PythonExpression(
        [
            '"',
            custom_config,
            '" != "" and ',
            'not "',
            custom_config,
            '".startswith("/")',
        ]
    ),
    if_value=PathJoinSubstitution([EnvironmentVariable("PWD"), custom_config]),
    else_value=custom_config,
)
```

The `custom_config` can then be passed as another parameter to the `ComposableNode` constructor.

```
node = ComposableNode(
    parameters=[
        {"uav_name": uav_name},
        {"use_sim_time": use_sim_time},
        {"custom_config": custom_config},
        {"default_config": this_pkg_path + "/config/default.yaml"},
    ],
)
```

Mind that the `mrs_lib::ParamLoader` searches for a parameter in the order of the config yaml files added to the `ParamLoader` object.
So, load the `custom_config` file before the `default.yaml`.
