---
layout: default
title: CloudCompare
parent: 3D model processing
grand_parent: Software
---

# CloudCompare (CC)

Tool for general work with pointclouds (including cartesian, color, and normal operations).
Used for data processing before meshing and texturing.

## Normals computing
- Recommend to use with *.e57* or *.ptx* data.
- `Edit -> Normals -> Compute`
- Local surface model
  - (best fit) Plane - robust to noise but very bad with sharp edges and corners
  - Triangulation - weak to noise but good with sharp edges
  - Quadric - very good for curvy surfaces
- Neighbors
  - Structured clouds can be associated to a grid structure. This grid structure can be used to determine the neighbors around each point. **However this is generally not better nor faster. This structure is however very useful to orient the normals properly.**
- Orientation
  - Use sensor(s) whenever possible - **recommend to use for .e57** 
  - If the cloud is associated to a grid structure, then this structure can be used to very guess the right normal orientation quickly and in a very robust way - **probably good to use with .ptx, but check**

## Smoothing colors
- `Edit -> Colors -> Convert to Scalar field`
- For each scalar field `{R, G, B}`
  - select scalar field in cloud properties
  - `Edit -> Scalar fields -> Gaussian filter`
    - lower kernel size: less smoothing (keeps more detail but filters less noise)
    - higher kernel size: more smoothing (keeps less detail but filters more noise)
- `Edit -> Scalar fields -> Convert to RGB`
  - be sure to select the correct R, G, B channels
  - the dynamic range is clipped by default: move the color arrows to reverse this
  - set alpha to 255

## Command line mode
[CC command line mode](https://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode) opens a way for scripting of most of the functions available within CC GUI.
**Scripting is faster than GUI and provides repeatability: prefer scripting over GUI.**
Example script is available [here](https://mrs.felk.cvut.cz/gitlab/NAKI/naki_postprocessing/blob/master/scripts/pointclouds/processPtxFiles.sh).
Example command merging two `.ptx` files, exporting them to `.ply`, sampling the data to resolution of 2 cm and exporting the sampled data to `.ply`:
```bash
CloudCompare -AUTO_SAVE OFF -O Setup249.ptx -O Setup250.ptx -MERGE_CLOUDS -C_EXPORT_FMT PLY -SAVE_CLOUDS FILE "merge_raw.ply" -SS SPATIAL 0.02 -SAVE_CLOUDS FILE "merge_raw_sampled_2cm.ply"
```

## Manual compilation
Get latest release version from [CC Github](https://github.com/CloudCompare/CloudCompare/releases).
Latest release on April 6, 2022: `v2.12.0`.

```bash
cd $GIT_PATH
git clone git@github.com:CloudCompare/CloudCompare.git
cd CloudCompare
git checkout v2.12.0

mkdir -p build
cd build

cmake \
  -DEIGEN_ROOT_DIR="$EIGEN_ROOT_DIR" \
  -DPLUGIN_GL_QEDL=ON \
  -DPLUGIN_GL_QSSAO=ON \
  -DPLUGIN_IO_QADDITIONAL=ON \
  -DPLUGIN_IO_QCORE=ON \
  -DPLUGIN_IO_QE57=ON \
  -DPLUGIN_IO_QPHOTOSCAN=ON \
  -DPLUGIN_IO_QPDAL=OFF \
  -DPLUGIN_IO_QRDB=ON \
  -DPLUGIN_IO_QRDB_FETCH_DEPENDENCY=ON \
  -DPLUGIN_IO_QRDB_INSTALL_DEPENDENCY=ON \
  -DPLUGIN_STANDARD_QANIMATION=ON \
  -DQANIMATION_WITH_FFMPEG_SUPPORT=OFF \
  -DPLUGIN_STANDARD_QBROOM=ON \
  -DPLUGIN_STANDARD_QCANUPO=OFF \
  -DPLUGIN_STANDARD_QCOMPASS=ON \
  -DPLUGIN_STANDARD_QCSF=ON \
  -DPLUGIN_STANDARD_QFACETS=ON \
  -DPLUGIN_STANDARD_QHOUGH_NORMALS=OFF \
  -DPLUGIN_STANDARD_QHPR=ON \
  -DPLUGIN_STANDARD_QM3C2=ON \
  -DPLUGIN_STANDARD_QPCV=ON \
  -DPLUGIN_STANDARD_QPOISSON_RECON=ON \
  -DPLUGIN_STANDARD_QSRA=ON \
  -DPLUGIN_STANDARD_QRANSAC_SD=OFF \
  -DPLUGIN_STANDARD_QPCL=ON\
  ..

cmake --build . --parallel
sudo cmake --install .
```

## Work in progress