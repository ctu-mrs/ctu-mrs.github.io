---
layout: default
title: How to compile
parent: Introduction
nav_order: 99
---

| :warning: **Attention please: This page is outdated.**                                                                                           |
| :---                                                                                                                                             |
| The MRS UAV System 1.5 is being released and this page needs updating. Plase, keep in mind that the information on this page might not be valid. |

We use [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) to manage our ROS workspaces.
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

Sometimes, cleaning the workspace can help with, e.g., corrupted binaries (which can happen on a drone, which is not usually shut down properly).
Cleaning might also help when dependencies changed too much.

Cleaning the whole workspace:
```bash
catkin clean
```

Cleaning a particular `<package>`:
```bash
catkin clean <package>
```
