# Converting from ROS2 bag file to ROS1 bag file

Good tool for this is the `rosags-convert` utility.

1. install rosbags package

```bash
pip install rosbags
```

2. Now you can convert the bag. Example:

```bash
rosbags-convert /rosbag2_2025_11_05-14_01_53/
```

This directory should contain `metadata,yaml` file. For this concrete example, the output should be in the file `rosbag2_2025_11_05-14_01_53.bag`.

If you are getting error like this:

```bash
ERROR: Reading source bag: Rosbag2 version 9 not supported; please report issue.
```

simply open the `/rosbag2_<timestamp>/metadata.yaml` file and lower the version number at the beginning of the file.

```yaml
rosbag2_bagfile_information:
  version: 9
```

In our case, just lowering it from `9` to `8` helped. This does not feel very correct, but if it works...
