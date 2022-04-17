---
layout: default
title: Testing
parent: 3D model processing
grand_parent: Software
---

# Testing
Other useful software that might help with the model processing in the whole pipeline.

## [Capturing Reality](https://www.capturingreality.com/)
* A professional software to create models from photos, laser scans, drone photos, etc.
* Offers [PPI licensing](https://www.capturingreality.com/Products) which allows to create the model and pay only for the result.
* Only works with Windows and require NVIDIA card with [CUDA 3.0+](https://support.capturingreality.com/hc/en-us/articles/115001524071-OS-and-hardware-requirements)
* Current issues:
    * To import `.ply` file, it has to contain [specific property](https://support.capturingreality.com/hc/en-us/community/posts/360009516459-Is-it-possible-to-import-unregistered-PLY-format-point-clouds-) which we cannot guarantee from Meshlab export.
    * The `.e57` format has some error while importing.
    * It seems only `.ptx` format will work as import.
    * I would recommend to watch first [YouTube](https://www.youtube.com/watch?v=y3aNUBckwnE&list=PL56jeA0rCS3LWuahdfIFWp1d0WDuEKVqe&index=28) series about the process. It seems it takes a lot of effort to do it. It cannot import final mesh, the whole process must be done inside.
    * It would require a powerful PC to do it right.
    * Im not sure about the final size, but it should be compressable.

## [Leica Cyclone 3DR](https://leica-geosystems.com/products/laser-scanners/software/leica-cyclone/leica-cyclone-3dr)
* Provides quite a bad quality mesh with a lot of imperfections. Meshlab does much better mesh.
* Texture is bad as well.
* Procesing is fast but not good quality. 
* It can do a nice texture from photos, but I did not have a time to look at it more deeply.
