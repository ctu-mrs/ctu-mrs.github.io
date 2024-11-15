---
title: Blender
---

# Blender
* Great tutorial how to do the [camera movements](https://www.youtube.com/watch?v=a7qyW1G350g&t=111s&ab_channel=Polyfjord)
* Always use the `.dae` model: fast loading and proper texture rendering
* I used Eeevee rendering mode, the fastest and sufficient with default settings

## Mesh parametrization
* Maps the mesh structure to the 2D plane to allow texturing.
* Import the `.ply` mesh file
* Change default `Object mode` to `Edit mode`
* Select all data with `a` key
* Open UV Mapping menu with `u` key
* Select `Smart UV Project`
* Keep the default values and press `OK`.
* Do something else. It takes a long time to process. Average an hour, depending on the model size. (250k faces ~1 hour 15 minutes, 500k faces ~3 hours). These values are only indicative, processing time is strongly hardware and model dependent!
* When finished, split the screen and change the `Editor Type` to `UV Editor`. It shows the texture parametrization.
* Export the file as `.ply` or `.obj`.
    * `.obj` format might be needed if `.ply` output is corrupted. It has to be exported from `Blender` with `Y` axis as `FORWARD` and `Z` axis as `UP`. Then import this file to `Meshlab`, skip `Convert PerVertex UV to PerWedge UV` because `.obj` file already contains `PerWedge UV` and export it in `.ply` format for next steps.

## Blender work with video recording
* Import .dae model
* Adding camera path by clicking on add -> curve -> path
* You can select path points by changing to Edit mode on left up corner and on the left side you can chose move, rotate or scale
* Adding camera by cliking on add -> camera
* Set camera on path by selecting camera, choose Object Constraint Properties tab on right panel, select add Object Constraint -> Follow Path
* Check fixed position
* You can set animation on 1st frame and right click on left panel on offset and set insert keyframe (you will add actual frame on loacation set by offset)

## Saving rendered animation
* Output properties tab on right panel, set location to save, set File Format on FFmpeg Video, set Container MPEG-4, set Output Quality high quality

## Blender with color pointcloud
* Use the [video](https://www.youtube.com/watch?v=kwpj7ZUtnac&ab_channel=Nicko16) to setup a plugin into Blender to allow pointcloud color visualization.
* All steps are described in the video.
