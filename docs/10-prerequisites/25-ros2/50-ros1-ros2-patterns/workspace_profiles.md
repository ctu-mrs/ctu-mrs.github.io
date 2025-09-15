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
Add the official mixin repository to install the default mixins:
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
```

Update the mixins:
```bash
colcon mixin update
```

List the mixins:
```bash
colcon mixin list
```

Build with particular mixin:
```bash
colcon build --mixin rel-with-deb-info
```

### Adding a custom Mixins
- Make a `custom.mixin` file that defines custom mixins(`JSON` format). Here we add a mixin for the `MRS_ENABLE_TESTING` flag.
```json
{
    "build": {
        "mrs-testing": {
            "cmake-args": ["-DMRS_ENABLE_TESTING=true",
                           "-DENABLE_TESTS=true"]
        },
    }
}
```

- Make an `index.yaml` file for `colcon mixin` to find all the defined mixins
```bash
mixin:
  - custom.mixin
```

- Add the file to the mixin repository and update the mixins:
```bash
colcon mixin add mrs file://<path-to-directory-with-mixin-index-file>/index.yaml
colcon mixin update mrs
```

## Using Mixins for build
- `colcon` uses configuration files for building, testing and installing packages. It's good practice to have a `colcon_defaults.yaml` in the root of your ROS2 workspace.
- The file can have different parameters as described at https://colcon.readthedocs.io/en/released/reference/verb/build.html. For eg. using a particular mixin and limiting the threads during build would look like
```yaml
build:
  parallel-workers: 8
  mixin:
    - rel-with-deb-info
    - mrs-testing
```
