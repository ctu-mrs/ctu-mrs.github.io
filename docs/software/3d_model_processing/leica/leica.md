---
layout: default
title: Leica Cyclone Register 360
parent: 3D model processing
grand_parent: Software
---

# Leica Cyclone Register 360 (LCR)
* Please press the question mark symbol **(?)** for detailed manual. You can find it in the top program bar.
* Nice [video](https://www.youtube.com/watch?v=AV0LPKowOXU&t=1s) guide.

## Import
1. Mobile: [Leica Cyclone Field 360](https://leica-geosystems.com/products/laser-scanners/software/leica-cyclone/leica-cyclone-field-360)
    - Allows to match scans on the field while scanning. Saves a lot of time.
    - Recommend to use an iPad instead of an Android device. Response and transfer time was much better for an iPad device. Works even for several high quality scans.
    - Matched scans can be transferred to the LCR directly from the device.
2. Static: PC
    - If scanned with sufficient density, the **Auto Cloud** option can be checked upon importing, which will automatically bind the scans together.

## Review and Optimize
* The precision of scan matching can be improved by editing the points used for cloud-to-cloud matching. These points can be removed from matching proces but still can be used for visualisation later on. Explained [here](https://youtu.be/AV0LPKowOXU?t=1574). Really useful if you have buildings and trees together (static and dynamic structures).
* If having scans between outdoor/indoor environment, it is better to process outdoor and indoor separately. Then merge them back together. Meanwhile, cut off the visual points going through windows/mirrors because they will create ghost points.
* There should be usually only **one bundle**. However, during the automatic bing process, there might be separate bundles created.
* Select two scans and click **Visual Alignment** if not satisfied with the automatic binding process. Scans can be aligned in xy-axes and z-axis separately.

## Cleaning
* Recommend to clean most of the noise in the Register 360 directly. Choose the **Bundle Cloud** mode and change the view of the camera to front/back/let/right/up/down position and cut off the noisy points with the selection tool. **You cannot undo one step. You can only restore all deleted points for particular setup.**
* **There is a new surface tool to cut off some anomalies, as humans, ghosts etc from final merge. Try it out** The surface detection works quite well. Recommend to see the tutorial in the [video](https://www.youtube.com/watch?v=AV0LPKowOXU&t=1s) guide at the end.

## Defining own cooridnate system
* The Register 360 allows to setup up a new coordinate system. Follow the tutorial in the help of LCR.
* It is important to change the used coordinate system in the field map overview.
* Then, the model can be rotated or shifted within the LCR without any change in the exported model.

## Limit Boxes
* If you change the view into **Bundle Cloud** mode, you can create **Limit Boxes**. These can be later on exported separately for more precise detail analyses.
* Useful to extract colors for a small part of the model in *.e57* or *.pts* format which allow for normals extraction.

## Export

### Useful formats
* **PTX** format is most suitable, because it is in the structured format for the normals computing. It is really useful for further mesh creation. However, Meshlab cannot import the single *.ptx* file, hence it is important to check the **separate files** button.
* **E57** format is similar to PTX, however Meshlab cannot import it. It is a structured format as well. It can be opened with CloudCompare software.
* **PTS** is useful for sharing the pointclouds with no intention for further meshing.

### E57
* It is possible to export as a single file with two variants
  * single point cloud (unstructered): can be decimated only with reduce cloud option providing average point spacing value, but misses the scanner position
  * separate setups (structured): can be decimated only with sub-sampling factor method, but contains the scanner position

### Decimation and limit box
- **PTX cannot be decimated in metrics, but only in some weird scale**. Use the highest PTX decimation as possible. It will be sufficient for the general model.
- If a small model is needed, just export it as pts in higher detail and compute normals manually.
- Decimation
  - lower resolution for robotic use (e.g., 5 cm)
  - higher resolution for meshing use (e.g., 1 mm)
