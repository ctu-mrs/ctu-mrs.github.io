---
layout: default
title: Whycon in simulation
parent: Simulation
---

# Whycon localization in simulation

TODO: this tutorial is probably outdated, pls update or remove

To install whycon, first clone the whycon repository into your workspace/src:

```bash
cd workspace/src
git clone https://github.com/lrse/whycon
```

We also need some dependencies:

```bash
rosdep install -y --from-path <path to whycon source package directory>
apt-get install ros-melodic-opencv3
```

Now we should be ready to compile, go to your workspace and:

```bash
source ~/.bashrc
catkin build
```

If your have problems with compiling, check the readme at [https://github.com/lrse/whycon](https://github.com/lrse/whycon) for more info.

We need to provide whycon with a topic with camera image and a camera info, which defines camera parameters (this topic is automatically generated in simulation).
To do this, we will modify the whycon launch file (whycon.launch, located in whycon/launch).
An example of a launch file is given here:

```xml
<launch>
   <arg name="name" default="whycon"/>
   <arg name="targets" default="1"/>
 
   <group ns="camera">
     <node pkg="image_proc" type="image_proc" name="image_proc"/>
   </group>
 
   <node name="whycon" type="whycon" pkg="whycon" output="screen">
     <param name="targets" value="1"/> 
     <param name="max_refine" value="5"/>
     <param name="max_attempts" value="1"/>
     <param name="outer_diameter" value="0.122"/>
     <param name="inner_diameter" value="0.05"/>
     <remap from="/camera/image_rect_color" to="/uav2/mobius_front/image_raw"/>
     <remap from="/camera/camera_info" to="/uav2/mobius_front/camera_info"/>
   </node>

   <node name="transformer" type="transformer" pkg="whycon" output="screen"/>
 </launch>
```

Image topic provided to whycon is in this case `/uav2/mobius_front/image_raw` and camera_info topic is `/uav2/mobius_front/camera_info`.

There are also some parameters, most notably inner and outer diameter, which define physical dimensions of the tracked circle.
Diameters in the launch file above correspond to the diameter of the circle used in simulation.
With target of this size, the visual detection will work to a distance of about ~4 meters.
To increase detection distance, you can scale up the whycon box.
Go to:

TODO old
```bash
cd ~/git/simulation/gazebo_files/rotors_description/urdf/
```
and edit file named f550.xacro. Search for the word "whycon" and you should find the whycon box macro. Here you can change the scale and position. For example, to double the box size, do this: mesh_scale="2 2 2"

After you made the edit, do:

```bash
cd ~/git/simulation/build
make install
```

To install the changes. Do not forget to change circle diameters in the whycon launch file!

Parameter "targets" sets how many targets will whycon try to track, if you set it to 1 and there are 2 targets in the camera image, it will only detect one. Parameters "max_attempts" and "max_refine" are there to make a trade-off between computational demand and detection quality. Check https://github.com/lrse/whycon/wiki/Reference for more details.

Now, we need to add cameras into the simulation, and targets on the drones that we want to track. Go to your simulation-launching script and find the spawn commands. You can add cameras and whycon targets like this:

```
- sleep 5; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth --enable-mobius-camera-front
- sleep 5; spawn 2 --run --delete --enable-rangefinder --enable-ground-truth --enable-whycon-box
```

```
--enable-whycon-box will add the whycon target.
--enable-mobius-camera-front will add a camera facing to the front, you can also add:
--enable-mobius-camera-back-left   --enable-mobius-camera-back-right --enable-mobius-camera-down
```

Now, run your simulation and run whycon:

```bash
roslaunch whycon whycon.launch
```

Check ROS topics:

```bash
rostopic list
```

Four new whycon topics should be visible, most notably:

/whycon/image_out - image with marked points detected by whycon, use rqt_image_view to view the image output.
/whycon/poses - poses of detected points.
