---
title: Compiling a workspace
pagination_label: Compiling a workspace with catkin
description: Compiling a workspace with catkin
---

Make sure you are located within the workspace when issuing the following commands.
If your workspace is sourced, you can `cd` to the workspace by:
```bash
roscd
```

## Compiling a workspace

To compile everything in `~/mrs_workspace`:
```bash
catkin build
```

## Compiling a specific package

To compile a particular `<package>`:
```bash
catkin build <package>
```

If you use Tomas's [linux-setup](https://github.com/klaxalk/linux-setup), you can use the key-binding `<leader>m` or call
```
:make
```
to compile the file you are editing.

## Cleaning the workspace

Cleaning the whole workspace:
```bash
catkin clean
```

Cleaning a particular `<package>`:
```bash
catkin clean <package>
```

