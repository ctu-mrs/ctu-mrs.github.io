---
title: Custom Real World File
pagination_label: Custom Real World File
description: How to create a custom real world file for the MRS system to fly in a new location
---

# Frames of Reference

As described in the [Frames of Reference](/docs/api/frames_of_reference) manual, the MRS system offers a flexible way to define the coordinate frames used in the system. This is enabled using geographical libraries in the real-world that map from latitude-longitude coordinates to the local Cartesian coordinate system. The MRS system uses the [GeographicLib](https://geographiclib.sourceforge.io/) library to handle these transformations.

# Custom Real World File

An example real world file is shown below. The file is divided into two main sections: `world_origin` and `safety_area`. The `world_origin` section defines the origin of the local Cartesian coordinate system. The `safety_area` section defines the safety area around the origin. The safety area is defined by a polygon in the horizontal plane and by the maximum and minimum heights in the vertical plane.


```bash
world_origin:

  units: "LATLON" # {"UTM, "LATLON"}

  origin_x: 50.0764041
  origin_y: 14.4180359

safety_area:

  enabled: true

  horizontal:

    # the frame of reference in which the points are expressed
    frame_name: "latlon_origin"

    # polygon
    #
    # x, y [m] for any frame_name except latlon_origin
    # x = latitude, y = longitude [deg]  for frame_name=="latlon_origin"
    points: [
      50.0765653, 14.4178914,
      50.0762102, 14.4179722,
      50.0763015, 14.4181847,
      50.0765825, 14.4181411,
    ]

  vertical:

    # the frame of reference in which the max&min z is expressed
    frame_name: "world_origin"

    max_z: 8.0
    min_z: 1.0
```

## Choice of coordinate system

There are two choices available for the coordinate system in the real world file: `LATLON` and `UTM`. The `LATLON` system uses latitude and longitude coordinates, while the `UTM` system uses the Universal Transverse Mercator (UTM) coordinate system. The choice of coordinate system is made by setting the `units` parameter in the `world_origin` section.

For example, the latitude and longitude for our university yard can be specified as ```50.076408, 14.418019``` in the `LATLON` system. The same point can be specified as ```458357.390, 5547288.346``` in the `UTM` system.

This origin leads to the defintion of a plane which converts from either `LATLON` or `UTM` to the local cartesian plane with co-ordinates in meters.


## Safety Area

Safety area is a polyhedron defined by vertically extruding a polygon to mark an area outside which the UAV should not fly. When specified and enabled, the system prevents ```goto``` commands and ```trajectory_references``` from taking the UAV outside the safety area. 

This area can either be defined in co-ordinate frames such as ```local_origin``` or ```world_origin``` which uses the origin specified in the world to define the polygon in meters, or in ```latlon_origin``` which uses the latitude and longitude to define the polygon in degrees.
