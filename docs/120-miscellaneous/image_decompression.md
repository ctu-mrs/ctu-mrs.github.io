# Republishing decompressed image stream and calibrating camera

To easily decompress incoming 'image_raw/compressed' stream into original 'image_raw' (e.g. for calibration), use the follwing command tested in ROS Jazzy:

```bash
run image_transport republish --ros-args --remap in/compressed:=/compressed_topic --remap out:=/decompressed_topic -p out_transport:=raw -p in_transport:=compressed
```

You can then calibrate your camera with the MRS checkerboard:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.052 --no-service-check --ros-args -r image:=/camera_raw
```
