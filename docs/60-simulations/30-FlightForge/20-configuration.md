---
title: Configuration
pagination_label: Configuring the Unreal Engine simulator
description: How to configure the Unreal Engine simulator
---

:::warning
This page is describing the upcomming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# Configuration

The FlightForge simulator is highly configurable. This page will guide you through the configuration options available to you.
The configuration is done using the ROS package `mrs_uav_unreal_simulation`.
The default configuration file is located in `mrs_uav_unreal_simulation/config/unreal_simulator.yaml`.

## Main Parameters

### Simulation and Clock Rates

-   `simulation_rate`: Defines the rate at which the simulation runs in Hz. The default value is 250.0 Hz.
-   `clock_rate`: Sets the rate of the simulation clock in Hz. The default value is 250.0 Hz.

### Realtime Factor

-   `realtime_factor`: Determines the desired real-time factor of the simulation. The default value is 1.0, which means the simulation runs in real-time.
-   `dynamic_rtf`: If set to `true`, the real-time factor is automatically adjusted so that the fastest sensor is always rendered at the desired rate. The default value is `true`.

### Collisions and Graphics

-   `collisions`: Enables or disables collisions in the simulation. The default value is `true`.
-   `graphics_settings`: Controls the level of graphics detail. Possible values are:
    -   `0` or `"low"`
    -   `1` or `"medium"`
    -   `2` or `"high"`
    -   `4` or `"epic"`
    -   `5` or `"cinematic"`
    -   The default value is `"medium"`.

### World and Environment

-   `world_name`: Specifies the environment or world to simulate. Available options are:
    -   `"valley"`
    -   `"forest"`
    -   `"infinite_forest"`
    -   `"warehouse"`
    -   `"cave"`
    -   `"erding_airbase"`
    -   `"temesvar"`
    -   `"electric_towers"`   
    -   `"race_1"` 
    -   `"race_2"`
    -   `"industial_warehouse"`
    -   `"service_tunnel"`
    -   `"dead_spruce_forest"`
    -   The default value is `"infinite_forest"`.
-   `weather_type`: Sets the type of weather in the simulation. Possible values are:
    -   `"sunny"`
    -   `"cloudy"` 
    -   `"foggy"` 
    -   `"rain"` 
    -   `"rain_light"` 
    -   `"rain_thunderstorm"`
    -   `"sand_dust_calm"`
    -   `"sand_dust_storm"`
    -   `"snow"`
    -   `"snow_blizzards"` 
    -   The default value is `"foggy"`.
-   `daytime`:
    -   `hour`: Sets the hour of the day (0-23). The default value is 7.
    -   `minute`: Sets the minute of the hour (0-59). The default value is 30.

### Forest Generation Settings

These settings apply only to the `"forest"` world.

-   `ueds_forest_density`: Adjusts the density of the forest clutter. Values range from 1 (high density) to 10 (low density). The default value is 6.
-   `ueds_forest_hilly_level`: Controls the hilliness of the terrain. Values range from 1 (totally flat) to 5 (most hilly). The default value is 3.

## Sensors

The `sensors` section allows you to configure various sensors available for the simulated drone.

### Rangefinder

-   `enabled`: Enables or disables the rangefinder. The default value is `true`.
-   `rate`: Sets the update rate of the rangefinder in Hz. The default value is 10.0 Hz.

### Lidar

-   `enabled`: Enables or disables the lidar sensor. The default value is `false`.
-   `rate`: Sets the update rate of the lidar in Hz. The default value is 10.0 Hz.
-   `horizontal_fov_left`:  Defines the left horizontal field of view in degrees. The default value is 180.0 degrees.
-   `horizontal_fov_right`: Defines the right horizontal field of view in degrees. The default value is 180.0 degrees.
-   `vertical_fov_up`: Defines the upper vertical field of view in degrees. The default value is 52.0 degrees.
-   `vertical_fov_down`: Defines the lower vertical field of view in degrees. The default value is 7.0 degrees.
-   `horizontal_rays`: Sets the number of horizontal rays. The default value is 128.
-   `vertical_rays`: Sets the number of vertical rays. The default value is 64.
-   `offset_x`, `offset_y`, `offset_z`: Define the positional offset of the lidar sensor from the drone's origin in meters.
-   `rotation_pitch`, `rotation_roll`, `rotation_yaw`: Set the rotational offset of the lidar sensor in degrees.
-   `beam_length`: Specifies the maximum range of the lidar beam in meters. The default value is 20.0 m.
-   `noise`:
    -   `enabled`: Enables or disables noise in the lidar measurements. The default value is `true`.
    -   `std_at_1m`: Sets the standard deviation of noise at 1 meter in meters. The default value is 0.01 m.
    -   `std_slope`: Defines the slope of noise increase with distance. The default value is 0.2.
-   `show_beams`:  Currently broken. Intended to visualize lidar beams.
-   `lidar_segmented`:
    -   `enabled`: Enables or disables segmented lidar data. The default value is `false`.
    -   `rate`: Sets the update rate of segmented lidar data in Hz. The default value is 10.0 Hz.
-   `lidar_intensity`:
    -   `enabled`: Enables or disables lidar intensity measurements. The default value is `false`.
    -   `rate`: Sets the update rate of lidar intensity data in Hz. The default value is 10.0 Hz.
    -   `values`: Defines intensity values for different materials (e.g., `grass`, `road`, `tree`, etc.).
    -   `noise`:
        -   `enabled`: Enables or disables noise in intensity measurements. The default value is `true`.
        -   `std_at_1m`: Sets the standard deviation of intensity noise at 1 meter. The default value is 0.59.
        -   `std_slope`: Defines the slope of intensity noise increase with distance. The default value is 0.81.

### RGB Camera

-   `enabled`: Enables or disables the RGB camera. The default value is `false`.
-   `rate`: Sets the update rate of the camera in Hz. The default value is 30.0 Hz.
-   `enable_hdr`: Enables or disables high dynamic range (HDR) imaging. The default value is `true`.
-   `enable_temporal_aa`: Enables or disables temporal anti-aliasing. The default value is `true`.
-   `enable_raytracing`: Enables or disables ray tracing. The default value is `true`.
-   `width`: Sets the image width in pixels. The default value is 640 px.
-   `height`: Sets the image height in pixels. The default value is 480 px.
-   `fov`: Sets the field of view of the camera in degrees. The default value is 120.0 degrees.
-   `offset_x`, `offset_y`, `offset_z`: Define the positional offset of the camera from the drone's origin in meters.
-   `rotation_pitch`, `rotation_roll`, `rotation_yaw`: Set the rotational offset of the camera in degrees.
-   `enable_motion_blur`: Enables or disables motion blur. The default value is `true`.
-   `motion_blur_amount`: Controls the amount of motion blur (0.0 - 1.0). The default value is 0.5.
-   `motion_blur_distortion`: Adjusts the distortion of motion blur (0-100). The default value is 50.0.
-   `rgb_segmented`:
    -   `enabled`: Enables or disables segmented image data. The default value is `false`.
    -   `rate`: Sets the update rate of segmented image data in Hz. The default value is 5.0 Hz.

### Stereo Camera

-   `enabled`: Enables or disables the stereo camera. The default value is `false`.
-   `rate`: Sets the update rate of the stereo camera in Hz. The default value is 10.0 Hz.
-   `enable_hdr`: Enables or disables high dynamic range (HDR) imaging. The default value is `true`.
-   `enable_temporal_aa`: Enables or disables temporal anti-aliasing. The default value is `true`.
-   `enable_raytracing`: Enables or disables ray tracing. The default value is `true`.
-   `baseline`: Sets the distance between the two stereo cameras in meters. The default value is 0.1 m.
-   `width`: Sets the image width in pixels. The default value is 640 px.
-   `height`: Sets the image height in pixels. The default value is 480 px.
-   `fov`: Sets the field of view of the camera in degrees. The default value is 90.0 degrees.
-   `offset_x`, `offset_y`, `offset_z`: Define the positional offset of the stereo camera from the drone's origin in meters.
-   `rotation_pitch`, `rotation_roll`, `rotation_yaw`: Set the rotational offset of the stereo camera in degrees.

## Frames

-   `world`:
    -   `name`: Specifies the name of the world frame. The default value is `"simulator_origin"`.
    -   `prefix_with_uav_name`: Determines whether to prefix the frame name with the UAV's name. The default value is `false`.
-   `fcu`:
    -   `name`: Specifies the name of the flight control unit (FCU) frame. The default value is `"fcu"`.
    -   `publish_tf`: Determines whether to publish the transform of this frame. The default value is `false`.
-   `rangefinder`:
    -   `name`: Specifies the name of the rangefinder frame. The default value is `"garmin"`.
    -   `publish_tf`: Determines whether to publish the transform of this frame. The default value is `false`.

## Randomization

-   `enabled`: Enables or disables randomization of initial conditions. The default value is `false`.
-   `bounds`:
    -   `x`, `y`, `z`: Define the bounds for randomization in each axis in meters. The default value is 15.0 m for each axis.

## Ground and Input

-   `ground`:
    -   `enabled`: Enables or disables a virtual ground plane. This ground is simulated outside of Unreal and should typically be turned off. The default value is `false`.
    -   `z`: Sets the height of the virtual ground plane in meters. The default value is 0.0 m.
-   `input_timeout`: Sets the timeout for input in seconds. The default value is 1.0 s.
-   `iterate_without_input`: Determines whether the simulation continues to iterate even without input. The default value is `true`.
- `g`: Sets the acceleration due to gravity in m/s^2. The default value is 9.81 m/s^2.
-   `individual_takeoff_platform`:
    -   `enabled`: Enables or disables a temporary takeoff platform placed under the spawn location. This platform disappears after the total thrust exceeds 90% of the hover thrust. The default value is `true`.
