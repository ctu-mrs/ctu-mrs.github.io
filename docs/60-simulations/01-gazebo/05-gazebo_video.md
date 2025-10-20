---
title: Videos from Gazebo
---

:::warning
This page is describing the upcoming ROS2 version of the MRS UAV System (however, it may be still outdated). If you are looking for ROS1 version of the docs, follow to https://ctu-mrs.github.io/docs/1.5.0/introduction/.
:::

# How to generate video from Gazebo

Videos from the Gazebo simulator can be recorded directly with the functionality of Gazebo GUI or by any screen-capturing software. However, these approaches do not produce videos of sufficient quality, so here we will provide an alternative approach which enables us to produce high-resolution videos.

## 1) Insert camera model in a simulation world

A model that contains camera plugin is called `static_camera` and its visual representation is a small white cube. When the model is present in the simulation world, it produces camera images on the topic `"/camera/rgb/image_raw"`, which can be visualized with rqt_image_view tool. The camera can be arbitrarily positioned to achieve the desired view.

## 2) Record rosbag with camera image topic

Once the camera is in the desired position, the camera image of the desired scene should be recorded (e.g., `ros2 bag record --topics /camera/rgb/image_raw`). Since the recording of raw images will produce enormous rosbags even for short videos, the recommended approach is to record compressed topics.

## 3) Generate sequence of images from rosbag

The following shell commands can be used for generation of a sequence of images from the rosbag. We strongly recommend creating a designated folder to contain the images, as there will be a lot of them.

```bash
ros2 bag play rosbag2_with_recorded_image.mcap < /dev/null &
ros2 run image_view image_saver --ros-args \
  -r image:=/camera/rgb/image_raw \
  -p filename_format:=frame%06i.png \
  -p image_transport:=compressed
```

`ros2 bag play` is called with the path to the rosbag which contains the recorded camera topic as an argument. This command can be skipped if you wish to use a live topic instead of a recorded rosbag.

`ros2 run image_view image_saver` is called with the following arguments: `"image"` is the name of the camera raw topic (e.g., `"/camera/rgb/image_raw"`), `"filename_format"` is the path to where the image sequence should be saved including the desired name of the files and image format (allowed types: jpg/png/pgm), and the `"image_transport"` stands for the type of recorded image - either `"raw"` or `"compressed"`. If `"compressed"` is set, image_view will use the compressed topic (e.g., `"/camera/rgb/image_raw/compressed"`).

## 4) Generate video sequence

From the sequence of images, the video can be created by ffmpeg tool with command:

```bash
ffmpeg -framerate 30 -i frame%06d.png -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
```

For more info and options see ffmpeg documentation [https://ffmpeg.org/ffmpeg.html](https://ffmpeg.org/ffmpeg.html).

## Advanced camera plugin options

## Notes and tips

1) The described approach can be used also for onboard cameras by skipping the first step.

2) If you need to record several views of the same scenario only for video, recording and replaying rosbag with some topics (e.g., "/uavx/control_manager/mpc_tracker/set_trajectory") can be a better approach than running code that generates trajectories repeatedly.

3) The length of the video does not have to be equal to length of the original simulation. This fact can be caused by different frame rate of the camera topic and framerate used in the ffmpeg command or by a varying frame rate of the camera topic.

4) Alternative approach using image_view package can be found [here](https://docs.ros.org/en/jazzy/p/image_view/doc/components.html#image-view-videorecordernode).

5) Resolution and other parameters of static camera can be modified in "model.sdf" file of the camera model.
