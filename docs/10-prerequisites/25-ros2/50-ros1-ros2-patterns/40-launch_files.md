---
title: Launch files
pagination_label: ROS2 Launch files
description: ROS2 Launch files
---
# Custom Configs
The lanuch files can use a `custom_config` launch arguement which can be used to supply customised parameters that overwrite the default parameters loaded by the node.
Add the following lines before declaring a `ComposableNode` object.
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
#     custom_config == "" => custom_config: ""
#     custom_config == "/<path>" => custom_config: "/<path>"
#     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
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
