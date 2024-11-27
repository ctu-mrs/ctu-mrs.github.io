---
title: Leica Cyclone Register 360
pagination_label: Working with Leica Cyclone Register 360
description: Working with Leica Cyclone Register 360
---

# Leica Cyclone Register 360 (LCR)

* Press the question mark symbol **(?)** for detailed manual in the top program bar.
* Good start [video](https://www.youtube.com/watch?v=AV0LPKowOXU&t=1s) guide.

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
* There should be usually only **one bundle**. However, during the automatic bind process, there might be separate bundles created.
* Select two scans and click **Visual Alignment** if not satisfied with the automatic binding process. Scans can be aligned in xy-axes and z-axis separately.

## Cleaning

* Recommend to clean most of the noise before the data export. Choose the **Bundle Cloud** mode and change the view of the camera to front/back/let/right/up/down position and cut off the noisy points with the selection tool. **You cannot undo one step. You can only restore all deleted points for particular setup.**
* **There is a new surface tool to cut off some anomalies, as humans, ghosts etc from final merge.** The surface detection works quite well. Recommend to see the tutorial in the [video](https://www.youtube.com/watch?v=AV0LPKowOXU&t=1s) guide at the end.

## Defining own cooridnate system

* The Register 360 allows to setup up a new coordinate system. Follow the tutorial in the help of LCR.
* It is important to change the used coordinate system in the field map overview.
* Then, the model can be rotated or shifted within the LCR without any change in the exported model.

## Limit Boxes

* If you change the view into **Bundle Cloud** mode, you can create **Limit Boxes**. These can be later on exported separately for more precise models.
* Useful to extract colors for a smaller part of the model only in *.e57* or *.pts* format.
* The *.e57* format allows to easier create normals in the **CloudCompare** software.

## Export

### E57

* Fomat contains **scan origins, images and points**.
* It is possible to export with two variants
  * **single point cloud (unstructered)**: can be decimated only with reduce cloud option providing average point spacing value. Useful to have equally distributed points in the scan, but misses the scanner position. Recommend to use **1mm** option. Have to be exported separatelly.
  * **separate setups (structured)**: can be decimated only with sub-sampling factor method, but contains the scanner position. Useful to gain complete information from scanning. The sub-sampling decimation doest not have equally distributed data, recommend to use only to have images and scan origins.
* Possible to open in both **Meshlab** and **CloudCompare** software.
* **Cloud Compare** is more useful to load and visualize all data.
* **Meshlab** does not support automatic normals creation and visualizing the scans origin and images.

### PTX

* Format contains **points and strange scan origins**.
* Scan origins are not located in the actual scan origin.
* Recommend to use the lowest decimation possible to have the most information. Exporting without decimation provides a huge file that has to be decimated anyway.
* **Meshlab** is more useful to load these data. It allows automatic normals computing. Useful for further mesh creation. However, **Meshlab** cannot import the single *.ptx* file, hence it is important to check the **separate files** button in LCR before exporting.

### **PTS**

* Format contains **point**.
* Useful for sharing the pointclouds with no intention for further meshing.
