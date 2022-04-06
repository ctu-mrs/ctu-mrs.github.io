---
layout: default
title: Texturing
parent: 3D model processing
grand_parent: Software
---

# Texturing
**A good quality texture can significantly visualy improve the low mesh model**
1. **Simple (primitive) texture: FAST AND EASY**
    * Go to **'Filters → Sampling → Vertex Attribute Transfer'**. Make sure the 'Transfer Color' option is checked and press 'Apply' to transfer the colour of the points onto the mesh.
    * Go to **'Filters → Texture → Parametrization: Trivial Per-Triangle'**. Set **Inter-Triangle border** to 5 for smoother transition between triangles. The higher the number, the bigger the texture will be. Press **'Apply'** to generate the faces from which the mesh texture will be created. If you receive an error about the inter-triangle border being too much, try increasing the **Texture dimension**.
    * Go to **'Filters → Texture → Vertex Color to Texture'**. Specify the texture file name and resolution for the mesh. The resolution should give the **Texture dimension** parameter from the previous step, e.g., **Texture width * Texture height = Texture Dimension**. Then press 'Apply' to create it.
2. **Good quality texture: SLOW BUT NICE**
  - Only for a single texture file and pointcloud color.
  - Recommend to use low quality *.ply* file about 10MB ~ 250k faces. If double the amount of faces (500k) the Blender process time will be really long.
  1. **[Blender](https://ctu-mrs.github.io/docs/software/3d_model_processing/blender)**
    - Import the *.ply* mesh file
    - Change default **Object mode** to **Edit mode**
    - Select all data with *a* key
    - Open UV Mapping menu with *u* key
    - Select **Smart UV Project**
    - Keep the default values and press *OK*.
    - Do something else. It takes a long time to process. Average an hour, depending on the model size. (250k faces ~1 hour 15 minutes, 500k faces ~3 hours)
    - When finished, split the screen and change the **Editor Type** to **UV Editor**. It shows the texture parametrization.
    - Export the file as *.ply*
      - Or (when some problems occure) export file as *.obj*, when exporting set **Y** as **FORWARD** axis and **Z** as **UP** axis. Then import this file to meshlab, skip **Convert PerVertex UV to PerWedge UV** because *.obj* already contain this and export it as *.ply*. 
  2. [Meshlab](https://ctu-mrs.github.io/docs/software/3d_model_processing/meshlab)
    - Import the Blender processed *.ply* file 
    - Run **Convert PerVertex UV to PerWedge UV** to convert Blender parametrization into Meshlab convention
    - Import the original *.ply* pointcloud with coloured vertices.
      - note: I have tried to use the high resolution mesh to transfer the color, but it has lower number of points giving worse texture quality
    - *Recommend step*: Hide the both models if running on PC with low graphic memory before further steps. Usually crash the Meshlab if not.
    - Open **Transfer: Vertex Attributes to Texture (1 or 2 meshes)** tool
      - Set the **Source Mesh** as the high quality pointcloud file
      - Set the **Target Mesh** as the parametrized texture file
      - Name the **Texture file** with *.png* extension and no path in the name.
      - Define the texture dimension in preferably multiplications of 2.
        - The higher the texture dimension, the better will be the final result. However, the larger will be the final file. Recommend to start with *8192x8192* res - *50MB* file. Even try the *32768x32768* res - *1GB* file. The difference will be visible. Depends on the application. The quality depends on the input pointcloud as well. The more detailed, the better.
        - Aftewards, it is possible to convert the texture to *.jpg* as shown below
        - *65536x65536* gives 2GB *.png* texture and takes ~ 3 hours
        - *32768x32768* gives 1GB *.png* texture and takes ~ 1 hour
      - The **Max Dist Search** parameter does not change the quality of the mesh. Keep it default.
      - Check **Fill Texture** checkbox. If you leave it unchecked, the final texture may contain visible fractures.
      - Click **Apply**
      - The process takes usually a lot of time. The cmd line will show some **QImage::pixel: coordinate (number,number) out of range** messages. It means it cannot fit the point from the pointcloud into the desired texture. However, this is not a problem. The texture from Blender is not predefined for specific dimension, hence arbitrary resolution will result in this message.
      - Save the final *.ply* file
  3. **Texture simplification**
    - Current texture is quite large and not that practicaly useful. Gazebo has its own [limitation](https://answers.gazebosim.org//question/1331/solved-jpeg-file-make-gazebo-die/) to load a large textures.
    - I recommend to convert it to *.jpg* with the [convert](https://linux.die.net/man/1/convert) tool: `convert input.png -quality 20 output.jpg`. The 20% quality is sufficient. The imagemagick tool might have some RAM/disk limitations set in default config */etc/ImageMagick-version/policy.xml*. Check it if you have errors. If yes put that limitations to coments.
    - For Gazebo use, you have to resize the image with `convert input -resample 16384x16384 output`. Otherwise, you will be unable to load the texture.
    - Do not forget to "resave" the *.ply* file with the converted texture in Meshlab.
      - Press *Export Mesh* icon
      - You should see your previous *.png* texture on the right side of the popup window.
      - Click *Rename Texture* then *Search Texture* and choose the *.jpg* one.
      - Save the *.ply* file.  
  4. **Several texture files**
    - A single texture file might be limiting the amount of details in the final model. Hence an 
4. **Texturing with raster images**
  - [link](https://wikis.utexas.edu/display/specify6/Texture+overlay+in+MeshLab), [video guide](https://www.youtube.com/playlist?list=PL60mCsep96Je1bzGrWnK-nL9pi95r7UqI)
  - Recommend to check these videos about image texturing. First is a [image alginment tool](https://www.youtube.com/watch?v=T7gAuI-LQ2w&ab_channel=MisterP.MeshLabTutorials) to visually align the image on the mesh. The second is [image parametrization and texturing](https://www.youtube.com/watch?v=OJZRuIzHcVw&ab_channel=MisterP.MeshLabTutorials) giving the final result.
  - If we do not have images for all parts of the model (churches), then it is necessary to cut out the parts of the mesh model, that we would like to color. Color them separately and then join the final models with separate textures.

6. EXPERIMENTAL: [Capturing Reality](https://www.capturingreality.com/)
    - A professional software to create models from photos, laser scans, drone photos, etc.
    - Offers [PPI licensing](https://www.capturingreality.com/Products) which allows to create the model and pay only for the result.
    - Only works with Windows and require NVIDIA card with [CUDA 3.0+](https://support.capturingreality.com/hc/en-us/articles/115001524071-OS-and-hardware-requirements)
    - Current issues:
        - To import **.ply** file, it has to contain [specific property](https://support.capturingreality.com/hc/en-us/community/posts/360009516459-Is-it-possible-to-import-unregistered-PLY-format-point-clouds-) which we cannot guarantee from Meshlab export.
        - The **.e57** format has some error while importing.
        - It seems only **.ptx** format will work as import.
        - I would recommend to watch first [YouTube](https://www.youtube.com/watch?v=y3aNUBckwnE&list=PL56jeA0rCS3LWuahdfIFWp1d0WDuEKVqe&index=28) series about the process. It seems it takes a lot of effort to do it. It cannot import final mesh, the whole process must be done inside.
        - It would require a powerful PC to do it right.
        - Im not sure about the final size, but it should be compressable.
7. EXPERIMENTAL: Leica Cyclone 3DR
    - Provides quite a bad quality mesh with a lot of imperfections. Meshlab does much better Mesh
    - Texture is bad as well.
    - Procesing is fast but not good quality. 
    - It can do a nice texture from photos, but I did not have a time to look at it more deeply.
    - We would have to buy it!

## Extra: Web format `.glb`
* [Original guide](https://mrs.felk.cvut.cz/gitlab/bednaj14/meshlab/blob/master/modely_report.pdf)
* The convert tool needs to have max *16384x16384* texture. Otherwise it will not work.
 
## Export
* **DAE** is the most suitable format for Gazebo.
* While exporting, you can choose to export following parameters:
    - Vert
        - **Color** - **UNCLICK ALWAYS** because color is not shown from vertices but from the texture
        - **Normals** - required always
        - **TexCoord** - **UNCLICK ALWAYS** because we used the Wedge TexCoord
    - Wedge
        - **TexCoord** - only for meshes with texture, this will increase the **DAE** filesize of about 2x times
* The *.dae* is exportable on 32 GB RAM system only for ~700 MB *.ply* files

## Smoothing
* Run **Smooth: Laplacian Vertex Color** and set number of iterations.
