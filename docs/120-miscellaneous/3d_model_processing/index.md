---
title: 3D model processing
---

# 3D Modeling guide

Purpose of this guide is to describe the steps of acquiring, processing, and presenting meshes and pointclouds through pipeline:

`Scanning with Leica -> Processing (cleaning, fixing colors, decimation) -> Meshing -> Texturing -> Export/Presenting on web`

## Pipeline

* [Leica Cyclone Register 360](https://ctu-mrs.github.io/docs/software/3d_model_processing/leica.html)
  * Import scans
  * Process (register, align, clean, limit)
  * Export data registered in shared coordinate system
    * Each full-point scan to separate `.e57` (unstructured)
    * All sparse-point scans with their scanning origins and color images to single `.e57` (structured)
* [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html) or custom scripts
  * Import separate `.e57` files
  * Clean points, [fix colors](https://ctu-mrs.github.io/docs/software/3d_model_processing/fixing_colors.html)
  * Merge all `.e57` files and decimate the result to desired resolution
* [Meshlab](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab.html) and/or [Blender](https://ctu-mrs.github.io/docs/software/3d_model_processing/blender.html)
  * Use to create mesh and texture
* Model [export](https://ctu-mrs.github.io/docs/software/3d_model_processing/export.html)

## Software versions

* [Meshlab](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab.html): 2022.02 or latest stable
* [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html): 2.11.3 compilled manually (how to in [readme](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html#manual-compilation))
* [Blender](https://ctu-mrs.github.io/docs/software/3d_model_processing/blender.html): latest stable
* [Leica Cyclone Register 360](https://ctu-mrs.github.io/docs/software/3d_model_processing/leica.html): latest

import DocCardList from '@theme/DocCardList';

<DocCardList />
