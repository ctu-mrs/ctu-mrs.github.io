---
slug: accurate-amsl-altitude
title: Update for accurate AMSL altitude positioning
authors: [petrlmat]
tags: [mrs]
---

We have implemented a major update in the MRS System, which changes how the AMSL (Above Mean Sea Level) altitude is calculated and used in the MRS system.
The update allows to fly to the exact AMSL altitude when using the RTK state estimator.

<!-- truncate -->

So far, the AMSL transformations (amsl_origin, utm_origin) published by `transform_manager` were based on the topic `hw_api/altitude`, which was based on the GNSS altitude reported by mavros/pixhawk. 
Now, when RTK is available (it is in the list of state estimators in custom config), the RTK altitude will be used instead.
The altitude is using the TF `fcu -> rtk_antenna` to transform the measurements received by the antenna into the fcu frame, which is typically offset vertically on the drones.
The `fcu -> rtk_antenna` TF was previously published from `sensors.launch` but I deleted the TF from the launch file because it is drone-specific and thus is the responsibility of the user to provide them in the tmux session. If the tf is not provided, transform manager will crash and inform the user to provide the tf.

Affected TFs: 
- `fcu -> amsl_origin` - Contains only the AMSL altitude (no XY position or orientation).
- `fcu -> utm_origin` - Contains the AMSL altitude, the XY position in the UTM frame (LatLon converted to meters), and the orientation.
 
EMLID uses the WGS84 ellipsoid model for altitude and we don't do any conversion, so our altitude is also in WGS84. Pixhawk uses geographiclib to convert GNSS WGS84 data into some geoid model (one of EGM2008, EGM96, EGM84), so the altitude in `hw_api/gnss` (pixhawk) and `hw_api/rtk` (EMLID) will differ by tens of meters (depending on the location).

This update concerns only EMLID RTK receivers. Flying with Holybro F9P had already accurate AMSL positioning and is not affected by this update as it is connected into pixhawk and not into NUC.

The update is currently on the Unstable PPA and master branch and coming to stable in the next release.
