---
layout: default
title: Standalone outside ROS
parent: MRS Simulator
grand_parent: Simulation
nav_order: 2
---

The MRS Multirotor Simulator contains a **header-only** implementation of the single UAV dynamics including a feedback controller pipeline.

## Headers

The headers are located in [include/mrs_multirotor_simulator/uav_system](https://github.com/ctu-mrs/mrs_multirotor_simulator/tree/master/include/mrs_multirotor_simulator/uav_system).
All the headers should be linked using relative includes so it should be possible to just copy the whole folder into your own code.

## Provided API

The API to the single UAV model is provided from:
```cpp
#include "uav_system.hpp"
```

### Constructors

Default constructor, uses parameters for Holybro x500 drone:
```cpp
UavSystem(void);
```

A constructor that accepts model parameters:
```cpp
UavSystem(const MultirotorModel::ModelParams& model_params);
```

A constructor that accepts model parameters and an initial spawn position:
```cpp
UavSystem(const MultirotorModel::ModelParams& model_params, const Eigen::Vector3d spawn_pos, const double spawn_heading);
```

### Stepping the simulator

The simulator step with a particular time step can be executed by:
```cpp
void makeStep(const double dt);
```

### Control inputs

The UAV accepts a variety of control inputs.
You can start controlling the drone by calling any of these functions:
```cpp
void setInput(const reference::Actuators& actuators);
void setInput(const reference::ControlGroup& control_group);
void setInput(const reference::AttitudeRate& attitude_rate);
void setInput(const reference::Attitude& attitude);
void setInput(const reference::TiltHdgRate& tilt);
void setInput(const reference::AccelerationHdgRate& acceleration);
void setInput(const reference::AccelerationHdg& acceleration);
void setInput(const reference::VelocityHdgRate& velocity);
void setInput(const reference::VelocityHdg& velocity);
void setInput(const reference::Position& position);

void setFeedforward(const reference::AccelerationHdgRate& cmd);
void setFeedforward(const reference::AccelerationHdg& cmd);
void setFeedforward(const reference::VelocityHdg& cmd);
void setFeedforward(const reference::VelocityHdgRate& cmd);
```

### Getting data

The full UAV state can be obtained by:
```cpp
MultirotorModel::State       getState(void);
```

The simulated accelerometer measurement can be obtained by:
```cpp
Eigen::Vector3d getImuAcceleration(void);
```

### Setting controller parameters

```cpp
void setMixerParams(const Mixer::Params& params);
void setRateControllerParams(const RateController::Params& params);
void setAttitudeControllerParams(const AttitudeController::Params& params);
void setVelocityControllerParams(const VelocityController::Params& params);
void setPositionControllerParams(const PositionController::Params& params);
```

### Other routines

A crash can be induced by:
```cpp
void crash(void);
```

External force can be applied to the UAV by:
```cpp
void applyForce(const Eigen::Vector3d& force);
```
