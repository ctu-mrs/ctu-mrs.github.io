---
layout: default
title: Texturing
parent: 3D model processing
grand_parent: Software
---

# Texturing
**A good quality texture can significantly improve the low quality mesh model**

We have experience with two ways of texturing. Pointcloud vertex color texturing and raster image texturing. Both will be described below.

## Pointcloud vertex color
These methods using pointcloud vertex color to create a full texture.
### Simple (primitive) texture: FAST AND EASY
* `Filters → Sampling → Vertex Attribute Transfer` Make sure the `Transfer Color` option is checked and press `Apply` to transfer the colour of the points onto the mesh.
* `Filters → Texture → Parametrization: Trivial Per-Triangle` Set `Inter-Triangle border` to 5 for smoother transition between triangles. The higher the number, the bigger the texture will be. Press `Apply` to generate the faces from which the mesh texture will be created. If you receive an error about the inter-triangle border being too much, try increasing the `Texture dimension`.
* `Filters → Texture → Vertex Color to Texture` Specify the texture file name and resolution for the mesh. The resolution should give the `Texture dimension` parameter from the previous step, e.g., `Texture width * Texture height = Texture Dimension`. Then press `Apply` to create it.

### Good quality texture: SLOW BUT NICE
* Recommend to use low quality `.ply` file about 10MB ~ 250k faces. If double the amount of faces (500k) the `Blender` processing time will be really long. However, this migt be different for various models, rather check by yourself.
 
#### `Blender`
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

#### `Meshlab`
  * Import the Blender processed `.ply` file. 
  * Run `Convert PerVertex UV to PerWedge UV` to convert `Blender` parametrization into `Meshlab` convention.
  * Import the original `.ply` pointcloud with coloured vertices.
    * I have tried to use the high resolution mesh to transfer the color, but it has lower number of points giving worse texture quality.
    * `Recommend`: Hide the both models if running on PC with low graphic memory before further steps. Usually crash the `Meshlab` if not.
  * Open `Transfer: Vertex Attributes to Texture (1 or 2 meshes)` tool.
    * Set the `Source Mesh` as the high quality pointcloud file.
    * Set the `Target Mesh` as the parametrized texture file.
    * Name the `Texture file` with `.png` extension and no path in the name.
    * Define the texture dimension in preferably multiplications of 2.
      * The higher the texture dimension, the better will be the final result. However, the larger will be the final file. Recommend to start with *8192x8192* res - *50MB* file. Even try the *32768x32768* res - *1GB* file. The difference will be visible. Depends on the application. The quality depends on the input pointcloud as well. The more detailed, the better.
      * *32768x32768* gives *1GB* `.png` texture and takes ~ 1 hour.
      * *65536x65536* gives *2GB* `.png` texture and takes ~ 3 hours.
      * Aftewards, it is possible to convert the texture to `.jpg` as shown below.
    * The `Max Dist Search` parameter does not change the quality of the mesh. Keep it default.
    * Check `Fill Texture` checkbox. If you leave it unchecked, the final texture may contain visible fractures.
    * Click `Apply`.
      * The process takes usually a lot of time. The cmd line might show `QImage::pixel: coordinate (number,number) out of range` messages. It means it cannot fit the point from the pointcloud into the desired texture. However, this is not a problem. The texture from `Blender` is not predefined for specific dimension, hence arbitrary resolution will result in this message.
    * Save the final result in `.ply` format.

### Texture simplification
  * It is recommended to convert the texture to `.jpg` format. Recommend to use either [Gimp](https://www.gimp.org/) or [convert](https://linux.die.net/man/1/convert) tool. Next steps will be described using `convert` tool.
    * `convert input.png -quality 20 output.jpg`
    * The 20% jpeg quality is sufficient. The `imagemagick` tool might have some RAM/disk limitations set in default config file `/etc/ImageMagick-version/policy.xml`. Check it if you have errors. If yes comment that limitations.
  * Do not forget to update the texture for current mesh. **READ THIS CAREFULLY, OTHERWISE YOU MIGHT ACCIDENTALY REWRITE THE TEXTURE FILE.**
    * Press `Export Mesh` icon.
    * You should see the original `.png` file name on the right side of the popup window.
    * Double click the texture name and type the name of the new updated texture file. **Make sure that there is no texture located in the same folder with such a name**.
    * Keep `Save Texture Files(s)` checkbox.
    * Save the `.ply` file. 
    * Replace the newly created texture with the one that you simplified to `.jpg` and keep the same name.
    * Open the `.ply` file again and validate, if the correct `.jpg` texture is applied. It can be checked in the `Export Mesh` option window.

## Texturing with raster images
The pointcloud vertex color might not be detailed enough for the whole model or it might help to improve part of the model with raster images.

### Improving part of the model
  * There is a nice text [guide](https://wikis.utexas.edu/display/specify6/Texture+overlay+in+MeshLab) describing the whole process. Moreover, there is a [video guide](https://www.youtube.com/playlist?list=PL60mCsep96Je1bzGrWnK-nL9pi95r7UqI) showing all steps in the video.
  * Recommend to check videos about **image texturing**. First is a [image alingment tool](https://www.youtube.com/watch?v=T7gAuI-LQ2w&ab_channel=MisterP.MeshLabTutorials) to visually align the image on the mesh. The second is [image parametrization and texturing](https://www.youtube.com/watch?v=OJZRuIzHcVw&ab_channel=MisterP.MeshLabTutorials) showing the final texture creation process and the result.
  * **IMPORTANT NOTICE**. It is necessary to cut out the parts of the mesh model, where images are missing. Then color the part of the model with images and the rest of the model with the pointcloud. Finally, join them into the final model having several texture files. Meshlab allows to merge several texture files in one `.ply` file.  
  * It might be useful to uncheck **Use distance weight** parameter. Otherwise, there will be dark color on some triangles.

### Create whole model 
  * **This method is not automatized, only proof of concept.** 
  * This process assume having precise position of the images w.r.t. the model.
  * In our case, you need the [`.e57`](https://ctu-mrs.github.io/docs/software/3d_model_processing/leica.html#e57) single file in **separate setups (structured)** variant. It will contain the raster images with precise position.
  * Use either [CloudCompare](https://ctu-mrs.github.io/docs/software/3d_model_processing/cloudcompare.html#extracting-images-and-sensor-positions) or [VoxelizeE57Files](https://mrs.felk.cvut.cz/gitlab/NAKI/naki_postprocessing/tree/master) package to extract images and scanner position.
  * **Recommend to save the MeshLab project `.mlp` as much as you can. MeshLab likes to crash**
  * Open the MeshLab and import all raster images `File->Import Raster...`
  * Select one of the raster images and right click `Export active raster cameras to file`. Output format `Agisoft xml` and click `Apply`.
  * Open the `cameras.xml` configuration file in some text editor and modify the `sensor` tag for each image as below.
```xml
<sensor id="keep the original" label="keep the original" type="frame">
  <resolution width="2048" height="2048"/>
  <property name="pixel_width" value="0.01"/>
  <property name="pixel_height" value="0.01"/>
  <property name="focal_length" value="10.235"/>
  <property name="fixed" value="false"/>
  <calibration type="frame" class="adjusted">
    <resolution width="2048" height="2048"/>
    <fx>1023.5</fx>
    <fy>1023.5</fy>
    <cx>1024</cx>
    <cy>1024</cy>
    <k1>0</k1>
    <k2>0</k2>
    <p1>0</p1>
    <p2>0</p2>
  </calibration>
</sensor>
```
* These values were extracted from `Camera Sensor` tag value from `.e57` files. You might notice, that the values are not exactly same as the one exported from CloudCompare, however this is how the Meshlab inteprets the imported value. If you have a different camera sensor or you would like to do it again, feel free to follow [camera settings](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab.html#camera-settings) guide.
