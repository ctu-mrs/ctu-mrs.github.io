---
layout: default
title: Blender
parent: 3D model processing
grand_parent: Software
---

# Blender
* Great tutorial how to do the [camera movements](https://www.youtube.com/watch?v=a7qyW1G350g&t=111s&ab_channel=Polyfjord)
* Always use the *.dae* model: fast loading and proper texture rendering
* I used Eeevee rendering mode, the fastest and sufficient with default settings

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
