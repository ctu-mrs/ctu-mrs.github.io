---
layout: default
title: Flying without mrs_uav_odometry
parent: The UAV system
---

# Flying without mrs_uav_odometry

**TODO**: somebody pls write the manual and test on yourself that it works

**Tomas's few suggestions what should be enough to perform:**
* you need to provide your own odometry and TFs (at least the "**fcu** to **world frame**" tf) and the frame names in the odmoetry (**frame_id** for the pose, **child_frame_id** for twist) need to be consistent
* [control_manager.yaml](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/control_manager.yaml) -> state_input := 0
* [control_manager.launch](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/launch/control_manager.launch) -> map odometry_in to your odometry topic
* [uav_manager.launch](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/launch/uav_manager.launch) -> map odometry_in to your odometry topic
* [uav_manager.yaml](https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/uav_manager.yaml) -> require_gain_manager := false, require_constraint_manager := false
  * gain and constraint managers require our custom OdometryDiagnostics, which is normally provided by [mrs_uav_odometry](https://ctu-mrs.github.io/docs/software/uav_core/mrs_uav_odometry/)
* Don't run the [core.launch](https://github.com/ctu-mrs/mrs_uav_general/blob/master/launch/core.launch) like we normally do (this would run [mrs_uav_odometry](https://ctu-mrs.github.io/docs/software/uav_core/mrs_uav_odometry/)), but run everything separately, like in the [standalone](https://github.com/ctu-mrs/simulation/tree/master/example_tmux_scripts/one_drone_gps_standalone) tmuxinator example (just without the **mrs_uav_odmoetry**, the **gain_manager** and the **constraint_manager**). Or modify the core launch and remove the respective includes.
* There might be a problem with a missing `odometry/height` topic, which might be needed to perform a normal landing, *pls investigate*.
