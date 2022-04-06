---
layout: default
title: 3D model processing
parent: Software
has_children: true
---

# 3D Modeling guide
Purpose of this guide is to describe the steps of acquiring, processing, and presenting pointclouds through pipeline:
`scanning with Leica -> processing (cleaning, fixing colors, decimation) -> meshing -> presenting on web`

## Pipeline
* [Leica Cyclone Register 360](https://ctu-mrs.github.io/docs/software/3d_model_processing/leica)
  * import scans
  * process (register, clean, limit)
  * export data registered in shared coordinate system
    * each full-point scan to separate `.e57` (unstructured)
    * all sparse-point scans with their scanning origins and color images to single `.e57` (structured)
* [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare) or custom scripts
  * import separate `.e57` files
  * clean points, [fix colors](https://ctu-mrs.github.io/docs/software/3d_model_processing/fixing_colors)
  * merge all `.e57` files and decimate the result to desired resolution
* [Meshlab](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab) and/or [Blender](https://ctu-mrs.github.io/docs/software/3d_model_processing/blender)
  * use to create mesh
* Create [textures](https://ctu-mrs.github.io/docs/software/3d_model_processing/texturing)

## Software versions

* [Meshlab](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab): 2021.05
* [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare): 2.11.3 compilled manually (how to in [readme](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare))
* [Blender](https://ctu-mrs.github.io/docs/software/3d_model_processing/blender): latest stable
* [Leica Cyclone Register 360](https://ctu-mrs.github.io/docs/software/3d_model_processing/leica): latest
