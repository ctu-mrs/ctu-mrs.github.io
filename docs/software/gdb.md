---
layout: default
title: Debugging with GDB
parent: Software
---

# Debugging with GDB

If you're experiencing crashes of your C/C++ ROS node/nodelet or if your program is not behaving as expected in general and you want to inspect it, you can reach for a debugger.
A debugger (namely GDB in our case) enables you to inspect the state of the program after a crash or at any point during the program runtime and is a very powerful tool for rooting out bugs.

## Preparing your node for GDB debugging

1. Make sure that the program is compiled with the flags `-O0 -g`[^1].
2. Run your node through the `debug_roslaunch` script by putting it to the `launch-prefix` in the launchfile[^2].
3. If you're debugging a nodelet, it is recommended to run it in standalone mode and not under a nodelet manager.

An example of how to set up your package like this is in the [ROS vision example package](https://github.com/ctu-mrs/example_ros_vision) (take a look in the `CMakeLists.txt` file and the launchfile).

## Debugging your node

You can setup breakpoints in the `~/.gdbinit` file.
When the program hits the breakpoint, it will stop and the GDB command console will open for you to enter commands (see the next section for useful GDB commands).
Breakpoints can also be conditioned e.g. on variable values etc. (use standard C++ syntax for the conditions).
The program will also break if it receives a signal from the operating system (e.g. because of a segmentation fault).
Lastly, you can break the program at any time by pressing `Ctrl+C` in the GDB tmux window.

When debugging, try to step through the code and see if it behaves as expected.
Print out variables or add them to the variable watch to check if their value makes sense to you.
You can also set up breakpoints to be triggered when a certain value changes.
This tutorial is meant to get you started, but feel free to look around online for more in-depth tutorials and explanations.
GDB is a very powerful tool, and if you learn how to use it properly, it will save you a lot of time.

## Useful GDB commands

A table of common useful GDB commands, which you'll probably need to debug your program, are listed below.
For a more exhaustive list, see the GDB manpages (`man gdb`) or any online tutorial.

| Command                  | Description                              | Comment                                  |
|--------------------------|------------------------------------------|------------------------------------------|
| b filename.cpp:310       | Breakpoint in `filename.cpp` at line 310 |                                          |
| bt                       | BackTrace                                |                                          |
| f #                      | change to Frame #                        | # = the number from bt                   |
| s                        | Step in function                         |                                          |
| n                        | step to the Next line                    |                                          |
| fin                      | FINish function                          | in case you accidentaly step into        |
| c                        | Continue                                 | resume program until breakpoint or crash |
| p #                      | Print variable                           | # = variable name                        |
| wh                       | open window with code (tui)              | actually sets Window Height              |
| tui enable / tui disable | open / close window with code (TUI)      | the official way of wh                   |
| focus cmd / focus src    | changes FOCUS in gdb tui                 | if you want to use arrows for cmd hist.  |
| up / down                | jumps in the frames UP/DOWN              |                                          |
| <Enter>                  | repeats the last command                 |                                          |
| u #                      | continue Until line #                    | # = the line number in the current file  |
| ref                      | REFresh the screen                       | in case of some visual problems          |
| `~/.gdbinit` file        | put pre-start settings in here           | an example is in the file                |

## Advanced debugging

If you're not satisfied with the basic debugging options GDB offers, note that it is very easily extendable and scriptable.
You can write your own plugins or search through the web for existing ones.
For example the [GDB dashboard](https://github.com/cyrus-and/gdb-dashboard) offers a better TUI for debugging (variable watching, breakpoint list, stack etc.).
You can play around with the `~/.gdbinit` file and tweak a lot of the settings there to better suit your needs.

[^1]: The `-O0` `gcc` flag disables optimizations, which makes debugging easier (although it potentially makes your program run slower). `-g` turns on generation of debugging symbols, which enables you to inspect code, print values of variables etc. during debugging.
[^2]: The `debug_roslaunch` script is a utility script, which will create a tmux split to separate output/output of your node and GDB in order to make debugging clearer. You can obtain it by installing the `uav_core` repository in case you don't have it.
