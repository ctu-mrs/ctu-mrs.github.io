---
layout: default
title: Custom drone
parent: Gazebo
grand_parent: Simulation
---

# Adding a custom drone to the simulation (tutorial)

![](fig/custom_drone.jpg)

In this tutorial, we will go through the process of adding a new vehicle to the MRS Gazebo simulation.
The tutorial covers the following steps:
* Custom directory setup
* Model template creation in Jinja
* Configuring a custom airframe for PX4
* Adding the custom drone to the MRS drone spawner
* Running the simulation
* Adding a custom optional sensor (configurable by the MRS drone spawner)
The complete example drone is available in the [example_custom_drone](https://github.com/ctu-mrs/example_custom_drone) repository.

## Custom directory setup
We highly recommend using a separate directory for your custom drones. The directory should be set up as a ROS package, which can be added into a ROS workspace and built with catkin.
The directory should follow this structure:
```yaml
example_custom_drone
  - package.xml
  - CMakeLists.txt
  - models
  - ROMFS
```

We use the `gazebo_ros_paths_plugin` which automatically updates the `GAZEBO_PLUGIN_PATH` and `GAZEBO_MODEL_PATH` when the package is built by catkin.

Make sure that your `package.xml` contains:
```xml
  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}"/>
  </export>
```

A minimalistic example of the full `package.xml` could be:
```xml
<?xml version="1.0"?>
<package format="2">

  <name>example_custom_drone</name>
  <version>0.0.1</version>
  <description>An example drone extending the MRS simulation</description>

  <maintainer email="todo@my.email">MY_NAME</maintainer>
  <license>BSD 3-Clause</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>cmake_modules</depend>
  <depend>mrs_uav_gazebo_simulation</depend>
  <depend>gazebo_ros</depend>

  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}"/>
  </export>

</package>
```

A corresponding minimalistic `CMakeLists.txt` has to be added for the package:
```cmake
cmake_minimum_required(VERSION 3.5)
project(example_custom_drone)

set(CATKIN_DEPENDENCIES
  cmake_modules
  mrs_uav_gazebo_simulation
  gazebo_ros
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  )
```

The `models` directory should contain separate directories for all custom drones. Let's create a directory called `my_drone`, which will serve as the root for our model. To setup a model root, change directory to `models/my_drone` and create a file called `model.config` with the following content:
```xml
<?xml version="1.0"?>
<model>
  <name>my_drone</name>
  <version>1.0</version>
</model>
```

The remaining directory with the name `ROMFS` must be placed in the root of our ROS package. This directory will hold all configs for PX4.

The package should be placed in a ROS [workspace](https://ctu-mrs.github.io/docs/software/catkin/managing_workspaces/managing_workspaces.html) and built at least once with catkin. The workspace does not have to be rebuilt after editing a `.sdf.jinja` file.


## Model template creation in Jinja
Jinja is a templating engine, which allows us to define SDF models with dynamically assigned variables that only become defined in runtime.
In essence the model for Gazebo is always created just at the time we need to place it into the world.

Let's create a jinja template for the `my_drone` model. The SDF Jinja template will be placed in `/models/my_drone/sdf/` and use the name `my_drone.sdf.jinja`. Make sure that the file **suffix** matches your suffix param from the [drone spawner config](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/config/spawner_params.yaml).

The basic model skeleton should look like this:
```
<?xml version="1.0" encoding="utf-8"?>
<sdf version='1.6'>

  {%- set root = 'base_link' -%}

  <model name="{{ spawner_args['name'] }}">

    <link name="{{ root }}">

    </link>

  </model>
</sdf>
```
In this example, `root` is set as a jinja variable, and will be used later in the code. The template will receive a dicitionary `spawner_args` from the MRS drone spawner. We will import some utilities from the `mrs_uav_gazebo_simulation`:
```
...
<sdf version='1.6'>
  {%- import 'mrs_robots_description/sdf/component_snippets.sdf.jinja' as mrs_components -%}
  {%- import 'mrs_robots_description/sdf/generic_components.sdf.jinja' as mrs_generic -%}
```
And use the multirotor_physics_macro to give our drone mass, inertia and a collision envelope:

```
...

  {%- set mass = 2.0 -%} {# [kg] #}
  {%- set body_radius = 0.1 -%} {# [m] #}
  {%- set body_height = 0.017 -%} {# [m] #}
  {%- set rotor_velocity_slowdown_sim = 0.0159236 -%}

  {%- set inertia_body_radius = 0.2 -%} {# [m] #}
  {%- set inertia_body_height = 0.075 -%} {# [m] #}

  {%- set body_ixx = mass * (3 * inertia_body_radius * inertia_body_radius + inertia_body_height * inertia_body_height) / 12 -%}
  {%- set body_ixy = 0 -%}
  {%- set body_ixz = 0 -%}
  {%- set body_iyy = mass * (3 * inertia_body_radius * inertia_body_radius + inertia_body_height * inertia_body_height) / 12 -%}
  {%- set body_iyz = 0 -%}
  {%- set body_izz = (mass * inertia_body_radius * inertia_body_radius) / 2 -%}

  ...

    <link name="{{ root }}">

      <!-- Body physics {-->
      {{ mrs_generic.multirotor_physics_macro(
        mass = mass,
        body_radius = center_board_radius,
        body_height = body_height,
        rotor_velocity_slowdown_sim = rotor_velocity_slowdown_sim,
        ixx = body_ixx,
        ixy = body_ixy,
        ixz = body_ixz,
        iyy = body_iyy,
        iyz = body_iyz,
        izz = body_izz)
      }}
      <!--}-->
```

Now we will add some visuals to the model, so that the drone body can be viewed in Gazebo. In this tutorial, we use a mix of geometric primitives and meshes. Since we use a `cylinder` visual several times in the project, it gives us a good opportunity to create a Jinja macro to attach a cylinder to the drone. To keep the base model file as clean as possible, let's create a new template called `custom_macros.sdf.jinja` in the `models/my_drone/sdf/` directory. The macro could look like this:

```
<?xml version="1.0" encoding="utf-8"?>
<sdf version='1.6'>

{%- macro visual_cylinder_macro(name, height, radius, color, x, y, z, roll, pitch, yaw) -%}
  {#- a generic macro to add a colored cylinder visual block. Must be placed inside a link block -#}
  <visual name="{{ name }}_visual">
    <pose>{{ x }} {{ y }} {{ z }} {{ roll }} {{ pitch }} {{ yaw }}</pose>
    <geometry>
      <cylinder>
        <length>{{ height }}</length>
        <radius>{{ radius }}</radius>
      </cylinder>
    </geometry>
    <material>
      <script>
        <name>Gazebo/{{ color }}</name>
        <uri>file://media/materials/scripts/gazebo.material</uri>
      </script>
    </material>
  </visual>
{%- endmacro -%}

</sdf>
```

Now let's go back to our `my_drone.sdf.jinja` and import the custom macros. We will use the newly created macro to add the central pieces of our drone - two flat cylinders stacked on top of each other with a small gap in between. This will serve as the center board of the frame.

```

  ...

  {%- import 'models/my_drone/sdf/custom_macros.sdf.jinja' as custom_macros -%}

  ...
  {%- set center_board_radius = 0.05 -%} {# [m] #}
  {%- set center_board_height= 0.002 -%} {# [m] #}
  {%- set center_board_offset_z = 0.01 -%} {# [m] #}

  {%- set arm_offset_z = 0.01 -%} {# [m] #}
  ...

  <!-- Body physics {-->
  ...
  <!--}-->

  <!-- Body visuals {-->

  <!-- Center boards {-->
  {{ custom_macros.visual_cylinder_macro(
    name = 'center_board_top',
    height = center_board_height,
    radius = center_board_radius,
    color = 'DarkGrey',
    x = 0,
    y = 0,
    z = center_board_offset_z,
    roll = 0,
    pitch = 0,
    yaw = 0)
  }}
  {{ custom_macros.visual_cylinder_macro(
    name = 'center_board_bottom',
    height = center_board_height,
    radius = center_board_radius,
    color = 'DarkGrey',
    x = 0,
    y = 0,
    z = -center_board_offset_z - arm_offset_z - body_height / 2,
    roll = 0,
    pitch = 0,
    yaw = 0)
  }}
  <!--}-->

  <!--}-->
```

Now we add the drone arms which use a mesh. We provide two meshes with this example: [arm and propeller](https://github.com/ctu-mrs/example_custom_drone/tree/main/models/my_drone/meshes).If placed in the model directory, the files can be accessed using uri `model://MODEL_NAME/RELATIVE_FILEPATH`

```
  {%- set arm_mesh_file = 'model://my_drone/meshes/arm.dae' -%}
  {%- set prop_mesh_file = 'model://my_drone/meshes/propeller.dae' -%}
  {%- set mesh_scale = '1 1 1' -%}
```

This model will have arms placed at unequal angles (i.e. body length > body width). **Notice that we use the math module from python**. The module has to be loaded into the jinja environment, which is then used to create a sdf file from the jinja template. In our case, the environment setup is handled by the MRS drone spawner, and only adds the `math` module. A macro to attach the mesh as a visual component is provided by [mrs_generic](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/models/mrs_robots_description/sdf/generic_components.sdf.jinja):

```
      ...

      <!-- Arms {-->
      {{ mrs_generic.visual_mesh_macro(
        name = 'front_right_arm',
        mesh_file = arm_mesh_file,
        mesh_scale = mesh_scale,
        color = 'DarkGrey',
        x = center_board_radius * math.cos(math.radians(-37)),
        y = center_board_radius * math.sin(math.radians(-37)),
        z = arm_offset_z,
        roll = 0,
        pitch = 0,
        yaw = math.radians(-37))
      }}

      {{ mrs_generic.visual_mesh_macro(
        name = 'front_left_arm',
        mesh_file = arm_mesh_file,
        mesh_scale = mesh_scale,
        color = 'DarkGrey',
        x = center_board_radius * math.cos(math.radians(37)),
        y = center_board_radius * math.sin(math.radians(37)),
        z = arm_offset_z,
        roll = 0,
        pitch = 0,
        yaw = math.radians(37))
      }}

      {{ mrs_generic.visual_mesh_macro(
        name = 'back_left_arm',
        mesh_file = arm_mesh_file,
        mesh_scale = mesh_scale,
        color = 'Indigo',
        x = center_board_radius * math.cos(math.radians(143)),
        y = center_board_radius * math.sin(math.radians(143)),
        z = arm_offset_z,
        roll = 0,
        pitch = 0,
        yaw = math.radians(143))
      }}

      {{ mrs_generic.visual_mesh_macro(
        name = 'back_right_arm',
        mesh_file = arm_mesh_file,
        mesh_scale = mesh_scale,
        color = 'Indigo',
        x = center_board_radius * math.cos(math.radians(217)),
        y = center_board_radius * math.sin(math.radians(217)),
        z = arm_offset_z,
        roll = 0,
        pitch = 0,
        yaw = math.radians(217))
      }}
      <!--}-->
```

Now we add the propellers. We utilize a propeller macro from [mrs_components](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation/blob/master/ros_packages/mrs_uav_gazebo_simulation/models/mrs_robots_description/sdf/component_snippets.sdf.jinja), which adds the motor physics and gazebo plugins, and also loads the mesh from a provided filepath. Note that propellers have their separate links, so they should be placed outside the root link block:

```
    ...
    {%- set motor_radius = 0.015 -%} {# [m] #}
    {%- set motor_height = 0.013 -%} {# [m] #}
    {%- set motor_offset_z = -0.01 -%} {# [m] #}

    {%- set motor_constant = 8.91 -%} {# [kg.m/s^2] #}
    {%- set moment_constant = 0.016 -%} {# [m] #}
    {%- set time_constant_up = 1.0 / 80.0 -%} {# [s] #}
    {%- set time_constant_down = 1.0 / 40.0 -%} {# [s] #}
    {%- set max_rot_velocity = 1 -%} {# [rad/s] #}
    {%- set rotor_drag_coefficient = 0.001 -%} {# orig 8.06428e-04 #}
    {%- set rolling_moment_coefficient = '1.0e-6' -%}

    {%- set prop_ixx = 0.0001 -%}
    {%- set prop_ixy = 0 -%}
    {%- set prop_ixz = 0 -%}
    {%- set prop_iyy = 0.0001 -%}
    {%- set prop_iyz = 0 -%}
    {%- set prop_izz = 0.0001 -%}
    ...
    </link>

    {# Propellers {--> #}
    {%- set prop_list = [
    {
    'motor_number': 0,
    'direction': 'cw',
    'x': (center_board_radius + arm_length) * math.cos(math.radians(-37)),
    'y': (center_board_radius + arm_length) * math.sin(math.radians(-37)),
    'z': body_height + arm_offset_z + motor_height + motor_offset_z + prop_offset_z,
    'mesh_files': [prop_mesh_file],
    'mesh_scale': prop_mesh_scale_cw,
    'color': 'Gold'
    },
    {
    'motor_number': 1,
    'direction': 'cw',
    'x': (center_board_radius + arm_length) * math.cos(math.radians(143)),
    'y': (center_board_radius + arm_length) * math.sin(math.radians(143)),
    'z': body_height + arm_offset_z + motor_height + motor_offset_z + prop_offset_z,
    'mesh_files': [prop_mesh_file],
    'mesh_scale': prop_mesh_scale_cw,
    'color': 'Gold'
    },
    {
    'motor_number': 2,
    'direction': 'ccw',
    'x': (center_board_radius + arm_length) * math.cos(math.radians(37)),
    'y': (center_board_radius + arm_length) * math.sin(math.radians(37)),
    'z': body_height + arm_offset_z + motor_height + motor_offset_z + prop_offset_z,
    'mesh_files': [prop_mesh_file],
    'mesh_scale': prop_mesh_scale_ccw,
    'color': 'Gold'
    },
    {
    'motor_number': 3,
    'direction': 'ccw',
    'x': (center_board_radius + arm_length) * math.cos(math.radians(217)),
    'y': (center_board_radius + arm_length) * math.sin(math.radians(217)),
    'z': body_height + arm_offset_z + motor_height + motor_offset_z + prop_offset_z,
    'mesh_files': [prop_mesh_file],
    'mesh_scale': prop_mesh_scale_ccw,
    'color': 'Gold'
    }
    ]
    -%}
    {{ mrs_components.propellers_macro(
      prop_list = prop_list,
      rotor_velocity_slowdown_sim = rotor_velocity_slowdown_sim,
      motor_constant = motor_constant,
      moment_constant = moment_constant,
      parent = root,
      mass = prop_mass,
      radius = prop_radius,
      time_constant_up = time_constant_up,
      time_constant_down = time_constant_down,
      max_rot_velocity = max_rot_velocity,
      rotor_drag_coefficient = rotor_drag_coefficient,
      rolling_moment_coefficient = rolling_moment_coefficient,
      meshes_z_offset = 0,
      prop_ixx = prop_ixx,
      prop_ixy = prop_ixy,
      prop_ixz = prop_ixz,
      prop_iyy = prop_iyy,
      prop_iyz = prop_iyz,
      prop_izz = prop_izz,
      spawner_args = spawner_args)
    }}
    {# <!--}--> #}
```

Make sure to follow the [PX4 airframe reference](https://docs.px4.io/main/en/airframes/airframe_reference.html) to assign correct orientation and motor number to the propellers. In this tutorial, we are building a **Quadrotor H** configuration (Note: a majority of quadrotors uses **Quadrotor X**).

The last part our drone needs to fly is the flight control unit (FCU). We will model the Pixhawk FCU with PX4 firmware. We can use an external mesh of the Pixhawk chasis, that is already included in mrs_uav_gazebo_simulation. The FCU includes the following sensors: GPS, IMU, magnetometer and barometer. It also creates a Gazebo-Mavlink interface, which needs to be connected to the correct ports (especially in case of multi-vehicle simulation). This is handled by the MRS drone spawner, which provides the ports in `spawner_args['mavlink_config']`:

```
  ...

  {%- set pixhawk_mesh_file = 'model://mrs_robots_description/meshes/sensors/pixhawk.dae' -%}

  ...
  {# Propellers {--> #}
  ...
  {# <!--}--> #}

  <!-- Pixhawk {-->
  {%- set imu_topic = '/imu' -%}
  {%- set mag_topic = '/mag' -%}
  {%- set baro_topic = '/baro' -%}
  {%- set lidar_topic = '/lidar' -%}

  <!-- Gazebo ground truth {-->
  {{ mrs_generic.gazebo_groundtruth_macro(
    home_latitude = 0,
    home_longitude = 0,
    home_altitude = 0)
  }}
  <!--}-->

  <!-- GPS {-->
  {{ mrs_generic.gazebo_gps_macro(
    gps_name = 'gps0',
    parent_link = root,
    update_rate = 10,
    gps_noise = true,
    gps_xy_random_walk = 2.0,
    gps_z_random_walk = 4.0,
    gps_xy_noise_density = '2.0e-4',
    gps_z_noise_density = '4.0e-4',
    gps_vxy_noise_density = 0.2,
    gps_vz_noise_density = 0.4,
    x = 0,
    y = 0,
    z = 0,
    roll = 0,
    pitch = 0,
    yaw = 0)
  }}
  <!--}-->

  <!-- Magnetometer {-->
  {{ mrs_generic.gazebo_magnetometer_macro(
    pub_rate = 100,
    noise_density = 0.0004,
    random_walk = 0.0000064,
    bias_correlation_time = 600,
    mag_topic = mag_topic)
  }}
  <!--}-->

  <!-- Barometer {-->
  {{ mrs_generic.gazebo_barometer_macro(
    baro_topic = baro_topic,
    pub_rate = 50,
    baro_drift_pa_per_sec = 0)
  }}
  <!--}-->

  <!-- Mavlink interface (customized) {-->
  {{ mrs_generic.gazebo_mavlink_interface_macro(
    imu_topic = imu_topic,
    mag_topic = mag_topic,
    baro_topic = baro_topic,
    lidar_topic = lidar_topic,
    mavlink_config = spawner_args['mavlink_config'])
  }}
  <!--}-->

  <!-- IMU {-->
  <!-- NOTE: IMU has to be last, otherwise the simulation is extremely slow! -->
  {{ mrs_generic.gazebo_imu_macro(
    imu_name = 'imu',
    parent_link = root,
    imu_topic = imu_topic,
    gyroscope_noise_density = 0.00018665,
    gyroscope_random_walk = 0.000038785,
    gyroscope_bias_correlation_time = 1000.0,
    gyroscope_turn_on_bias_sigma = 0.0087,
    accelerometer_noise_density = 0.00186,
    accelerometer_random_walk = 0.006,
    accelerometer_bias_correlation_time = 300.0,
    accelerometer_turn_on_bias_sigma = 0.1960,
    x = 0,
    y = 0,
    z = 0,
    roll = 0,
    pitch = 0,
    yaw = 0)
  }}
  <!--}-->

  <!--}-->
```
