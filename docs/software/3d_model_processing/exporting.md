---
layout: default
title: Exporting
parent: 3D model processing
grand_parent: Software
---

# Exporting
Details, how to export the final model.

## Web
* Please follow the [original guide](https://mrs.felk.cvut.cz/gitlab/bednaj14/meshlab/blob/master/modely_report.pdf) and the scripts inside.
* Recommend to install `NodeJS` with [snapcraft](https://snapcraft.io/node) tool. The `apt` version for `Ubuntu` does not contain the up-to-date version.
* The convert tool needs to have max *16384x16384* texture. Otherwise it will not work. Validate this sentence!

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
