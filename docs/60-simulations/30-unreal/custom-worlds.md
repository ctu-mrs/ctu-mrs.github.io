---
title: Custom simulation world
pagination_label: Creating custom worlds for the Unreal Engine simulator
description: How to create a custom world for the Unreal Engine simulator 
---

# How to recreate a real place in the Unreal Engine

Create a new Unreal Engine project or open an existing project with the FlightForge UE plugin.
If you are creating a new project make sure to place the [FlightForge UE plugin](https://github.com/ctu-mrs/unreal_engine_simulator_blueprint) in the `Plugins` folder of the project, and enable it in the project settings.


To recreate a real place you need to start by acquiring the appropriate data.
The data can be obtained from various sources, such as:
1. [CUZK](https://ags.cuzk.cz/geoprohlizec/?k=11273) - Czech Republic data
2. [Geodaten](https://geodaten.bayern.de/opengeodata/index.html) - Bavaria Germany data
3. [OpenStreetMap](https://www.openstreetmap.org/) - worldwide data (requires API key)
4. [Google Maps API](https://developers.google.com/maps/documentation) - worldwide data (requires API key, paid service)
 
Whichever source you choose, you will need the following data:
- **Heightmap** - a grayscale image representing the terrain elevation
- **Satellite image** - a colored image representing the terrain texture
- **Other** - 3D models of buildings, trees, etc.

## Heightmap

The heightmap will be used to create the terrain in the Unreal Engine.
The heightmap has to be a square image with a resolution of 
