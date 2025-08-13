---
title: Mixin AKA Workspace Profiles
pagination_label: ROS2 workspace profiles mixin
description: ROS2 workspace profiles
---

# ROS2 workspace profiles

## Setting work-space wide configurations

## Mixin
A `mixin` is a tag given to a set of parameters that you may want to pass to `colcon` during build (maybe used with other `colcon` commands).

### Adding Mixins to `colcon`
Add the official mixin repository:
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
```

Update the mixins:
```bash
colcon mixin update
```

Show the mixins:
```bash
colcon mixin show
```

Build with particular mixin:
```bash
colcon build --mixin rel-with-deb-info
```

### Adding a custom Mixins
- Make a `build_types.mixin` file that defines custom mixins(`JSON` format)
```json
{
    "build": {
        "debug": {
            "cmake-args": ["-DCMAKE_BUILD_TYPE=Debug",
                            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                            "-DCMAKE_CXX_FLAGS='-std=c++17 -Og'",
                            "-DCMAKE_C_FLAGS='-Og'"]
        },
        "rel-with-deb-info": {
            "cmake-args": ["-DCMAKE_BUILD_TYPE=RelWithDebInfo",
                            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                            "-DCMAKE_CXX_FLAGS='-std=c++17'"]
        },
        "release": {
            "cmake-args": ["-DCMAKE_BUILD_TYPE=Release",
                            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                            "-DCMAKE_CXX_FLAGS='-std=c++17'"]
        }
    }
}
```
- Make an `index.yaml` file for `colcon mixin` to find all the defined mixins
```bash
mixin:
  - build_types.mixin
```
- Add the file to the mixin repository and update the mixins:
```bash
colcon mixin add mrs file://<path-to-directory-with-mixin-index-file>/index.yaml
colcon mixin update mrs
```
Add these lines to your `.zshrc` or `.bashrc` to automatically access the mixins with each new shell.

## Using Mixins for build
- `colcon` uses configuration files for building, testing and installing packages. It's good practice to have a `colcon_defaults.yaml` in the root of your ROS2 workspace.
- The file can have different parameters as described at https://colcon.readthedocs.io/en/released/reference/verb/build.html. For eg. using a particular mixin and limiting the threads during build would look like
```yaml
build:
  parallel-workers: 8
  mixin:
    - rel-with-deb-info
```