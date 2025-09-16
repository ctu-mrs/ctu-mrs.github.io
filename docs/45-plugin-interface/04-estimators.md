---
title: Estimator plugins
pagination_label: API for the estimator plugins
description: API for the estimator plugins
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::


# The Estimator plugin interface
The Estimator plugin takes in IMU sensor data(Orientation and Angular velocity) and Controller command and provides UAV state consisting of position, velocity and acceleration. Diagram below shows the heirarchical structure defining base classes and derived classes.

The estimator plugin is compiled as ROS plugins with the interface defined by the estimator manager. A estimator plugin from any ROS package can be loaded dynamically by the estimator manager without it being present during estimator manager's compile time. Loaded estimators can be switched by the estimator manager in mid-flight, which allows safe testing of new estimator and adds flexibility to the MRS UAV system.

Linear kalman filter is used for state estimation. It is important to set appropriate value of covariance parameter in config file and it depends on sensor specifications.

![estimator_dependency_tree-Page-1.drawio](https://hackmd.io/_uploads/BJq_KRBolx.png)

# Estimator Output
* Position
* Velocity
* Acceleration

# Example Estimator Plugin

* The estimation manager, gain manager, constraint manager and transform manager as well loads parameters from `custom_config.yaml` before loading any plugins.  

* Generic partial estimators for individual states like altitude, lateral and heading are already implemented. These partial estimators are recommended to be customised using appropriate modification to parameters in `custom_config.yaml`. These parameters are give below.

```
mrs_uav_managers:
  # |----------------------------------------------------------|
  # |                   Estimation manager                     | 
  # |----------------------------------------------------------|
   
  estimation_manager:

    # loaded state estimator plugins
    state_estimators: [
      "example_estimator_plugin",
    ]

    initial_state_estimator: "example_estimator_plugin" # will be used as the first state estimator
    agl_height_estimator: "" # only slightly filtered height for checking min height (not used in control feedback)
    
    # if set to true it will publish this topics.
    debug_topics:
      input: true
      output: true
      state: true
      covariance: false
      innovation: false
      correction: true
      correction_delay: false
      diagnostics: false
  
    example_estimator_plugin:
      # should this be in the config folder with private and public config parameters.
      # address should be same as the name of plugin mentioned in xml file
      address: "example_estimator_plugin/ExampleEstimator"

      requires: # data required from the hw api
        gnss: true
        imu: false
        distance_sensor: false
        altitude: true
        magnetometer_heading: false
        position: true
        orientation: true
        velocity: true
        angular_velocity: true

      ## for heading estimator
      hdg_hw_api: # namespace of the heading estimator

        max_flight_z: 100.0 # [m] maximum allowed flight Z (in the estimator frame)

        topics:
          orientation: "hw_api/orientation" # without uav namespace
          angular_velocity: "hw_api/angular_velocity" # without uav namespace

      estimators: # the names of the partial estimators
        lateral:
          name: "lat_gps"
        altitude:
          name: "alt_baro"
        heading:
          name: "hdg_hw_api"
          passthrough: true # if true, then heading is not estimated but passed through from the orientation topic

      ## for lateral estimator
      lat_gps: # namespace of the lateral estimator

        max_flight_z: 100.0 # [m] maximum allowed flight Z (in the estimator frame)

        innovation:
          limit: 1.0 # [m] innovation limit that will trigger action
          action: "eland" # {"eland", "switch", "mitigate"}

        hdg_source_topic: "gps_baro/hdg_hw_api/output" # [mrs_uav_state_estimation/EstimatorOutput]

        repredictor: # repredictor for correct fusion of delayed measurements
          enabled: false

        process_noise: # process noise covariance (Q)
          pos: 0.1 # position state
          vel: 1.0 # velocity state
          acc: 1.0 # acceleration state

        corrections: [
          "pos_hw_api"
        ]

        pos_hw_api:
          state_id: 0 # 0 - position, 1 - velocity, 2 - acceleration
          noise: 0.01 # measurement noise covariance (R)
          noise_unhealthy_coeff: 100.0 # covariance gets multiplied by this coefficient when correction is unhealthy (R)
          transform:
            enabled: false # if true, from_frame and to_frame need to be provided for transformation.
          message:
            type: "geometry_msgs/PointStamped"
            topic: "hw_api/position" # without uav namespace
            limit: 
              delay: 0.5 # [s] messages with higher delay will flag correction as unhealthy
              time_since_last: 0.5 # [s] larger time step between messages will flag correction as unhealthy

          processors: ["tf_to_world"] # types of processors attached to this measurement

          tf_to_world:
            gnss_topic: "hw_api/gnss"
      
      ## for altitude estimator      
      alt_baro: # namespace of the altitude estimator

        max_flight_z: 100.0 # [m] maximum allowed flight Z (in the estimator frame)

        innovation:
          limit: 1.0 # [m] innovation limit that will trigger action
          action: "eland" # {"eland", "switch", "mitigate"}

        repredictor: # repredictor for correct fusion of delayed measurements
          enabled: false

        process_noise: # process noise covariance (Q)
          pos: 1.0 # position state
          vel: 1.0 # velocity state
          acc: 1.0 # acceleration state

        corrections: [
          # positional correction only makes the estimator unstable, vel_hw_api is not a derivative of pos_hw_api, vel_hw_api_only works best
          "vel_hw_api"
        ]

        transform:
          enabled: false

        vel_hw_api:
          state_id: 1 # 0 - position, 1 - velocity, 2 - acceleration
          noise: 0.001 # measurement noise covariance (R)
          noise_unhealthy_coeff: 100.0 # covariance gets multiplied by this coefficient when correction is unhealthy (R)
          message:
            type: "geometry_msgs/Vector3Stamped"
            topic: "hw_api/velocity" # without uav namespace
            limit: 
              delay: 1.0 # [s] messages with higher delay will flag correction as unhealthy
              time_since_last: 0.5 # [s] larger time step between messages will flag correction as unhealthy
          transform:
            enabled: false
          body_frame: true
          processors: [] # types of processors attached to this measurement
  
      topics:
        orientation: "hw_api/orientation" # orientation passthrough
        angular_velocity: "hw_api/angular_velocity" # angular velocity passthrough

      override_frame_id:
        enabled: false # if true, custom frame_id can be provided instead of the default "[estimator_name]_origin"
        frame_id:  

  ## |----------------------------------------------------------|
  ## |                      Gain manager                        | 
  ## |----------------------------------------------------------|
  gain_manager:
    estimator_types: [
      "example_estimator_plugin"
    ]
    # list of allowed gains per odometry mode
    allowed_gains:
      example_estimator_plugin: ["supersoft", "soft"]
    
    default_gains:
      example_estimator_plugin: "soft"
  
  ## |----------------------------------------------------------|
  ## |                   Constraint manager                     | 
  ## |----------------------------------------------------------|
  constraint_manager:
    estimator_types: [
      "example_estimator_plugin" 
    ]

    allowed_constraints:
      example_estimator_plugin: ["slow", "medium", "fast"]

    default_constraints:
      example_estimator_plugin: "slow"

  ## |----------------------------------------------------------|
  ## |                      UAV manager                         | 
  ## |----------------------------------------------------------|
  uav_manager:

    midair_activation:

      after_activation:
        controller: "Se3Controller"
        tracker: "MpcTracker"

  ## |----------------------------------------------------------|
  ## |                   Tranform manager                       | 
  ## |----------------------------------------------------------|
  transform_manager:
      example_estimator_plugin:
        odom_topic: "odom" # name of the topic (expects nav_msgs/Odometry topic type)
        tf_from_attitude: # used for transforming velocities before full transform is available
          enabled: true
          attitude_topic: "attitude" # name of the attitude topic(expects geometry_msgs/QuaternionStamped topic type)
        namespace: "mrs_uav_state_estimators/example_estimator_plugin" # the namespace of the topic (usually the node that publishes the topic)
        utm_based: true # if true, will publish tf to utm_origin
        inverted: true # publish as inverted tf (the default for inverted mrs tf tree convention)
        republish_in_frames: [] # the odometry message will be transformed and republished in the specified frames
        custom_frame_id:
          enabled: false
        custom_child_frame_id:
          enabled: false
```
