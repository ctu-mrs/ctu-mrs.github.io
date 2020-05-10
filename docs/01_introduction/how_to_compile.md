---
layout: default
title: Buttons
parent: Introduction
---

# How to compile

As you have already learned in the [file structure](file_structure) page, the environment consists technically of three ROS workspaces.
I wrote __technically__, because one of them ([qudrotor_control_binaries](qudrotor_control_binaries)) is non-compillable and contains just pre-compilled binaries.
The other two (~/mrs_workspace and ~/workspace) are kept separate to split our packages from yours.
The main advantage of splitting them to two compilation spaces is the compile time.
Because ~/workspace **extends** ~/mrs_workspace, you can still use everything produced by ~/mrs_workspace, but you do not have to compile our packages everytime you want to compile (or clean/recompile) yours.
In the same sense, ~/mrs_workspace extends [qudrotor_control_binaries](qudrotor_control_binaries).

For this reason, **make sure** you have the ~/mrs_workspace compiled (hopefully without errors) before you try to compile your code in ~/workspace.

## How to compile a workspace?

Following text applies to any workspace, but the examples are shown on the ~/mrs_workspace.

To compile everything, first got to the workspace's root
```bash
cd ~/mrs_workspace
```
and run general compilation command.
```bash
catkin build
```
It can also be run from any subdirectory of the workspace.

If you encounter a *compilation error*, which looks similar to this: `c++: internal compiler error: Killed (program cc1plus)`, your computer just ran out of RAM (which might happen if you have > 4 CPU cores but only <= 8 GB of RAM).
Please compile the _mavros_ packages first by `catkin build mavros` and then proceed to compile the whole workspace by `catkin build` again.

## Compiling a specific package

To compile a specific package only, run
```bash
catkin build <package_name>
```
from anywhere in the workspace.

If you happen to be editing your code with **vim**, issue command
```
:make
``` 
to compile the particular package, you are editing.

## Cleaning the workspace (for a fresh compilation)

Sometimes, a cleaning can help with e.g. corrupted binaries (which can happen on a drone, which is not usually powered-off properly).
To clean the whole workspace, run
```bash
cd ~/mrs_workspace
catkin clean
```
To clean a single package, run
```bash
cd ~/mrs_workspace
catkin clean <package_name>
```
