---
layout: default
title: Texturing
parent: 3D model processing
grand_parent: Software
---

# Texturing
**A good quality texture can significantly improve the low quality mesh model**
## Simple (primitive) texture: FAST AND EASY
* `Filters → Sampling → Vertex Attribute Transfer` Make sure the `Transfer Color` option is checked and press `Apply` to transfer the colour of the points onto the mesh.
* `Filters → Texture → Parametrization: Trivial Per-Triangle` Set `Inter-Triangle border` to 5 for smoother transition between triangles. The higher the number, the bigger the texture will be. Press `Apply` to generate the faces from which the mesh texture will be created. If you receive an error about the inter-triangle border being too much, try increasing the `Texture dimension`.
* `Filters → Texture → Vertex Color to Texture` Specify the texture file name and resolution for the mesh. The resolution should give the `Texture dimension` parameter from the previous step, e.g., `Texture width * Texture height = Texture Dimension`. Then press `Apply` to create it.
## Good quality texture: SLOW BUT NICE
* Only for a single texture file and pointcloud color.
* Recommend to use low quality `.ply` file about 10MB ~ 250k faces. If double the amount of faces (500k) the `Blender` processing time will be really long.
* Blender
  * Import the `.ply` mesh file
  * Change default `Object mode` to `Edit mode`
  * Select all data with `a` key
  * Open UV Mapping menu with `u` key
  * Select `Smart UV Project`
  * Keep the default values and press `OK`.
  * Do something else. It takes a long time to process. Average an hour, depending on the model size. (250k faces ~1 hour 15 minutes, 500k faces ~3 hours)
  * When finished, split the screen and change the `Editor Type` to `UV Editor`. It shows the texture parametrization.
  * Export the file as `.ply` or `.obj`.
    * `.obj` format might be needed if `.ply` output is corrupted. It has to be exported from `Blender` with `Y` axis as `FORWARD` and `Z` axis as `UP`. Then import this file to `Meshlab`, skip `Convert PerVertex UV to PerWedge UV` because `.obj` file already contains `PerWedge UV` and export it in `.ply` format for next steps. 
