---
title: Fixing colors
pagination_label: Fixing colors in pointclouds
description: Fixing colors in pointclouds
---

# Fixing colors

## Most color noise can be smoothed by Gaussian filter

- Use [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html#smoothing-colors)

## Highly bad data can be fixed with custom pipeline

Export
  - 1 structured `.e57` file with *N* low-res clouds, origins and images
  - *N* unstructured `.e57` files with full-point clouds.

1. Merge unstructured data with [VoxelizeE57FilesByOriginDistance](https://mrs.fel.cvut.cz/gitlab/NAKI/naki_postprocessing/tree/master)
  - voxelizes data to desired resolution
  - final voxel stores by default a point with the lowest distance from the scanning origin (origin parsed from the structured `.e57` file)
  - filters out measurements with respect to the sensor accuracy
  - produces a binary `.pcd` file
2. Smooth colors using [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html#smoothing-colors)
