---
layout: default
title: Fixing colors
parent: 3D model processing
grand_parent: Software
---

# Fixing colors

Assuming 1 structured `.e57` with origins and images, and *N* unstructured `.e57` files with full-point clouds.

1. Merge unstructured data with [VoxelizeE57FilesByOriginDistance](https://mrs.felk.cvut.cz/gitlab/NAKI/naki_postprocessing/tree/master)
  - voxelizes data to desired resolution
  - final voxel stores by default a point with the lowest distance from the scanning origin (origin parsed from structured `.e57` file
  - produces a binary `.pcd` file
2. Smooth colors using [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html/#smoothing-colors)


