---
layout: default
title: Videos from Gazebo
parent: Simulation
---

# How to generate video from Gazebo

Videos from Gazebo simulator can be recorded directly with the functionality of Gazebo GUI or by any screen-capturing software. However, these approaches do not produce videos of sufficient quality, so we provide a description of an alternative approach which enables to produce high-resolution videos. 

## 1) Insert camera model in a simulation world 
A model that contains camera plugin is called `static_camera` and its visual representation is a small white cube. When the model is present in the simulation world, it produces camera images on topic `"/camera/rgb/image_raw"`, which can be visualized with rqt_image_view tool. The camera can be arbitrarily positioned to achieve the desired view. 

## 2) Record rosbag with camera image topic 
Once the camera is in the desired position, the camera image of the desired scene should be recorded (e.g. `rosbag record /camera/rgb/image_raw`). Since the recording of raw images will produce enormous rosbags even for short videos, the recommended approach is to record compressed topics.
 
## 3) Generate sequence of images from rosbag 
Following launch file can be used for generation of a sequence of images from the rosbag. We strongly recommend creating a designated folder to contain the images, as there will be a lot of them. You can also paste the code in between the `<launch>` tags into an existing launch file:

```
<launch>
  <arg name="camera_topic" default="/camera_raw_image_topic"/>
  <arg name="bag_file" default="$(env PWD)/bagfile_with_recorded_image.bag"/>
  <arg name="image_location" default="$(env PWD)/frame%06i.png"/>
  <arg name="image_type" default="compressed"/>

  <node pkg="rosbag" type="play" name="rosbag" args="$(arg bag_file)"/>
  <node name="extract" pkg="image_view" type="extract_images" respawn="false" output="screen">
    <remap from="image" to="$(arg camera_topic)"/>
    <param name="filename_format" value="$(arg image_location)"/>
    <param name="image_transport" value="$(arg image_type)"/>
  </node>
</launch>
```

The `"camera_topic"` is the name of the camera raw topic (e.g. `"/camera/rgb/image_raw"`), `"bag_file"` is the absolute path to the rosbag which contains recorded camera topic, `"image_location"` is the absolute path to folder where the image sequence should be saved including the desired name of the files and image format (allowed types: jpg/png/pgm), and the `"image_type"` stands for the type of recorded image - either `"raw"` or `"compressed"`. If `"compressed"` is set, the image_view node will expect compressed topic (e.g. `"/camera/rgb/image_raw/compressed"`).   

## 4) Generate video sequence 
From the sequence of images, the video can be created by ffmpeg tool with command: 

`
ffmpeg -framerate 30 -i frame%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
`

For more info and options see ffmpeg documentation (https://ffmpeg.org/ffmpeg.html).

## Advanced camera plugin options

## Notes and tips
1) The described approach can be used also for onboard cameras by skipping the first step.
2) If you need to record several views of the same scenario only for video, recording and replaying rosbag with some topics (e.g. "/uavx/control_manager/mpc_tracker/set_trajectory") can be a better approach than running code that generates trajectories repeatedly.
3) The length of the video does not have to be equal to length of the original simulation. This fact can be caused by different frame rate of the camera topic and framerate used in the ffmpeg command or by a varying frame rate of the camera topic. 
4) Alternative approach using image_view package can be found at http://wiki.ros.org/image_view in section 4.3 video_recorder.
5) Resolution and other parameters of static camera can be modified in "model.sdf" file of the camera model.   
