---
layout: default
title: mrs_mavros_interface
parent: uav_core
grand_parent: Software
---
# MRS Mavros Interface

| Build status | [![Build Status](https://github.com/ctu-mrs/mrs_mavros_interface/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/mrs_mavros_interface/actions) | [![Build Status](https://github.com/ctu-mrs/mrs_mavros_interface/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_mavros_interface/actions) |
|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|

An interface between [MRS UAV core](https://github.com/ctu-mrs/uav_core) and [Mavros](http://wiki.ros.org/mavros).
Historically, it provided conversion between Mavros's and our conventions, but there is no need for them now.
Now it only contains the **MavrosDiagnostics** node.
It is kept around for future opportunities if we need to add a conversion.

## MavrosDiagnostics

* Reports independently on changes of the UAV state (arming, offboard..) in a terminal - very useful when digging through logs.
* Publishes custom diagnostics message.
