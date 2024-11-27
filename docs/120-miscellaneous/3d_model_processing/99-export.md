---
title: Exporting models
pagination_label: Exporting the final model
description: Exporting the final model
---

# Export

Details, how to export the final model.

## General formats

Most of the times, the models are saved in following formats

### Polygon File Format - `.ply`

* Mainly used MeshLab format.
* Allows binary format of the file which saves a lot of space on the data storage.
* Allows to store all kind of data into this format.

### Wavefront - `.obj`

* Useful to share data with other software as Blender
* More structured format for sharing.
* Unfortunatelly bigger file size.

## Web

* The basic guide will be described here. Please follow the [original guide](https://mrs.felk.cvut.cz/gitlab/bednaj14/meshlab/blob/master/modely_report.pdf) and the scripts inside for detailed information.
* Download the [obj2optimizedGlb.sh](https://mrs.felk.cvut.cz/gitlab/bednaj14/meshlab/blob/master/obj2optimizedGlb.sh) script.
* Recommend to install `NodeJS` with [snapcraft](https://snapcraft.io/node) tool. The `apt` version for `Ubuntu` does not contain the up-to-date version.
* The convert tool needs to have max *16384x16384* texture. Otherwise it will not work.
  * *Note: In general, it is better to have several lower size texture files than a single large one.*
  * **Raster texture warning:** If you create a texture from raster images, check the *gltf-pipeline* tool parameter *quantizePositionBits* to have value lower than 25, e.g. 20. Otherwise, the model will not be possible to visualize.
* Run `./obj2optimizedGlb.sh input_file.obj output_file.glb`
  * Parameters `quantizePositionBits`,`quantizeNormalBits`, `quantizeTexcoordBits`, `quantizeColorBits` and `quantizeGenericBits` are set to max values to provide best quality model. Feel free to lower the values to decrease the `.glb` file size.
* Upload the `.glb` file in [glTF Viewer](https://gltf-viewer.donmccurdy.com/) to view the result.

## Gazebo

* The `.dae` format is most suitable for Gazebo.
* Gazebo has its own [limitation](https://answers.gazebosim.org//question/1331/solved-jpeg-file-make-gazebo-die/) to load a large textures.
* The image has to be resized with `convert input.jpg -resample 16384x16384 output.jpg`. Otherwise, Gazebo will be unable to load the texture.
* While exporting, you can choose to export following parameters:
    * Vert
        * `Color` - UNCLICK ALWAYS because color is not shown from vertices but from the texture.
        * `Normals` - required always.
        * `TexCoord` - UNCLICK ALWAYS because we used the Wedge TexCoord.
    * Wedge
        - `TexCoord` - only for meshes with texture, this will increase the `.dae` filesize of about 2x times.
* The `.dae` is exportable on 32 GB RAM system only for ~700 MB `.ply` files.
