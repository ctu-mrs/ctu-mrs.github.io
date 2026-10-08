# Republishing decompressed image stream

To easily decompress incoming 'image_raw/compressed' stream into original 'image_raw' (e.g. for calibration), use the follwing command tested in ROS Jazzy:

```bash
run image_transport republish --ros-args --remap in/compressed:=/compressed_topic --remap out:=/decompressed_topic -p out_transport:=raw -p in_transport:=compressed
```

