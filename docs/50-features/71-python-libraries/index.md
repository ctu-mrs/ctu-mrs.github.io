---
title: Python libraries
pagination_label: MRS Python libraries
description: MRS Pyhon libraries
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

## ROS2 launch system libraries

### RemappingsCustomConfigParser

**RemappingsCustomConfigParser** is a ROS2 launch component that can pull topic, service and action names remappings from the custom config and add them to the ROS2 Node.

<Button label="🔗 ctu-mrs/mrs_lib repository" link="https://github.com/ctu-mrs/mrs_lib" block /><br />

<Button label="🔗 ctu-mrs/mrs_lib documentation" link="https://ctu-mrs.github.io/mrs_lib/" block /><br />

#### Example

The simplest case is when your config looks like this:

```yaml
remappings:
  /uav1/image_raw: /topic123

input:
  camera_frame_id: uav1/rgb
  camera_world_frame_id: simulator_origin
  subsample_factor: 1 # Process one out of this many images.
  ...
```

Then you just have to add **RemappingsCustomConfigParser** to your launchfile after the node definition, give it the node and the path to the config:

```python
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser

...

node = ComposableNode(
    package='flame_ros',
    plugin='flame_ros::FlameRos',
    name='flame_ros',
    namespace='uav1',
    parameters=[{'use_sim_time': True},
                LaunchConfiguration('custom_config')],
    extra_arguments=[{'use_intra_process_comms': True}],
    remappings=[('/uav1/image_raw', '/uav1/rgb/image_raw'),
                ('/uav1/camera_info', '/uav1/rgb/camera_info')]
)

parser = RemappingsCustomConfigParser(node, LaunchConfiguration('custom_config'))
```

If you want to be more specific, you can name the remappings and paste specific remappings to the specific node. You have to wrap your remappings into the namepsace:

```yaml
flame:
  remappings:
    /uav1/image_raw: topic123
```

and you also have to put the namespace argument to the **RemappingsCustomConfigParser**:

```python
parser = RemappingsCustomConfigParser(node, LaunchConfiguration('custom_config'), 'flame')
```

**RemappingsCustomConfigParser** overrides the remappings that were given to the node in the launchfile.

You can also use the different namespacing, just like you do for ordinary remappings definition. For example, if you have node with namespace **/uav1/flame_ros**, you can remap topic/service/action in three different ways:

* `/uav1/image_raw: topic123` will result in `/uav1/topic123`
* `/uav1/image_raw: /topic123` will result in `/topic123`
* `/uav1/image_raw: ~/topic123` will result in `/uav1/flame_ros/topic123`

This also applies to the original topic. In this case, it would be enough to not specify any namespace:

* `image_raw: topic123` will result in `/uav1/topic123`

The original topic is automatically prefixed (`image_raw -> /uav1/image_raw`) just like the new topic is (`topic123 -> /uav1/topic123`).