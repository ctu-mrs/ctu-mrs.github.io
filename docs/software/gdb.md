---
layout: default
title: Debugging with GDB
parent: Software
---

| :warning: **Attention please: This page needs work.**                                                                                             |
| :---                                                                                                                                              |
| The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid. |

# Debugging with GDB

If you're experiencing crashes of your C/C++ ROS node/nodelet or if your program is not behaving as expected in general and you want to inspect it, you can reach for a debugger.
A debugger (namely GDB in our case) enables you to inspect the state of the program after a crash or at any point during the program runtime and is a very powerful tool for rooting out bugs.

## Preparing your node for GDB debugging

1. Make sure that the program is compiled with the flags `-O0 -g`[^1].
2. Run your node through the `debug_roslaunch` script by putting it to the `launch-prefix` in the launchfile[^2].
3. If you're debugging a nodelet, it is recommended to run it in standalone mode and not under a nodelet manager.

An example of how to set up your package like this is in the [waypooint flier](https://github.com/ctu-mrs/mrs_core_examples) (take a look in the `CMakeLists.txt` file and the launchfile).

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

| Command                      | Description                              | Comment                                                                                     |
|------------------------------|------------------------------------------|---------------------------------------------------------------------------------------------|
| `b filename.cpp:310`         | Breakpoint in `filename.cpp` at line 310 |                                                                                             |
| `bt`                         | BackTrace                                | use `bt full` for a more detailed `bt`                                                      |
| `f #`                        | change to Frame `#`                      | `#` = the number from `bt`                                                                  |
| `s`                          | Step in function                         |                                                                                             |
| `n`                          | step to the Next line                    |                                                                                             |
| `fin`                        | FINish function                          | in case you accidentaly step into a fun.                                                    |
| `c`                          | Continue                                 | resume program until breakpoint or crash                                                    |
| `p #`                        | Print variable                           | `#` = variable name                                                                         |
| `wh`                         | open window with code (TUI)              | actually sets Window Height                                                                 |
| `tui enable` / `tui disable` | open / close window with code (TUI)      | the official way of `wh`                                                                    |
| `focus cmd` / `focus src`    | changes FOCUS in gdb TUI                 | if you want to use arrows for cmd hist.                                                     |
| `up` / `down`                | jumps in the frames UP/DOWN              |                                                                                             |
| `<Enter>`                    | repeats the last command                 |                                                                                             |
| `u #`                        | continue Until line `#`                  | `#` = the line number in the current file                                                   |
| `ref`                        | REFresh the screen                       | in case of some visual problems                                                             |
| `cv_imshow #`                | display an OpenCV image `#`              | requires [a plugin](https://github.com/ViktorWalter/gdb-imshow) (installed with `uav_core`) |
| `~/.gdbinit` file            | put pre-start settings in here           | an example is in the file                                                                   |

## Advanced debugging

### Attaching to a running process

You can also debug an already running program.
Typically, this is useful when you encounter a deadlock in your program or another state that is hard to reproduce.
To do this, first you need to know the PID (Program IDentifier) of the program that you want to attach GDB to.
You can find that using `htop`, `pidof`, `pgrep` or any other command.
For example, to find the PID of the process in which the `ControlManager` nodelet is running, you can use the command
```
pgrep -fia controlmanager
```
When you know the PID of the process you want to attach to, use the command
```
sudo gdb -p PID
```
to do that (it has to be done using superuser privileges - if a normal user could do this, that would be quite a security concern).
The program will be paused by GDB after attaching - use the `continue` command (or `c` for short) to resume normal execution and use `sudo gdb -p PID -ex c` to issue the command automatically immediately after attaching if you want to avoid the pause.
To detach gdb from the program, use the `detach` command.

### Debugging a deadlocked program
*Note: Inspired by [this blog post](https://codeistry.wordpress.com/2016/10/24/gdb-find-the-thread-which-has-locked-the-mutex/).*

If the deadlock ocurred when no debugger is attached to the program, you can attach to a running program using the method described above.
When you have gdb prompt available, a good way to start debugging the deadlock is to list all threads using
```
info threads
```
and finding threads that are waiting on a mutex.
These will typically list their current callframe function as `__lll_lock_wait ()` or something similar.
A command to list the complete backtrace of all threads that you may also find useful is
```
thread apply all bt
```
When you find a thread that is waiting on a mutex, switch to its context using
```
thread <thread number>
```
Then, you can print details of the mutex that the thread is waiting for (you may need to change the current callframe using the `up`, `down` or `frame` commands).
Specifically, you're looking for the current owner of the mutex.
Typically, you need to change the frame up until you hit the frame with the lock and then print the dereferenced mutex pointer using a command like
```
p *mutex
```
In the output, you should see several pieces of information, but most importantly the PID of the owning thread.
When you know the PID of the owner, you can change context to the corresponding thread (thread PIDs are listed using the `info threads` command) and check why is that thread deadlocked (it will typically be also waiting on some mutex that is locked by something else).
This way, you should be able to find the cause of the deadlock.

### Debugging a program that crashed without GDB attached

You can even debug a program that crashed and was not launched with an attached debugger.
However, you have to make sure that coredumping is actually enabled by running
```
ulimit -a | grep core
```
If the configured core file size is zero, no coredump will be created.
To enable coredump, run (*will change settings for the current terminal only!*)
```
ulimit -c unlimited
```
Now, when a program crashes, its core will be autmatically saved a file named `core` in the current path (this is the default behaviour).
If you need to differentiate between coredumps from different programs, enable coredumping with PID:
```
echo 1 | sudo tee /proc/sys/kernel/core_uses_pid
```
Now, the coredump will be named `core.<PID>`, so a new crash will not overwrite an old coredump.

Finally, after your program crashes, you can debug it using simply the command
```
gdb <path to the program> <path to the core>
```
Note that for ROS nodelets, `<path to the program>` should typically be something like `~/workspace/devel/lib/libYourNodelet.so` and `<path to the core>` is by default `/var/lib/apport/coredump/core.<filepath+filename>.<unimportant number>.<unimportant hash>.<PID>.<unimportant number>` in Ubuntu 20.04 when using `apport`.

It is possible that the `apport` service may need to be enabled using `sudo service apport enable` (you can check its status using `sudo service apport status`).
Other possible locations of the coredump are the current path and `~/.ros/<core dump filename>`.

Using the `ulimit -c unlimited` command will only enable coredump until logging out, then the default behaviour will be reset.
To permanently enable coredump for all users, edit the file `/etc/security/limits.conf` (as root) and add these two lines:
```
root - core unlimited
*    - core unlimited
```

## Getting debugging symbols for the C++ standard library

By default, the `libstdc++` regrettably doesn't come installed with debugging symbols in Ubuntu 20.04 (this shouldn't be a problem for 22.04 and newer releases).
This means, that you can't use e.g. `std::cout::operator<<()` and other functions from the standard library while in GDB, which may be quite limiting.
[Here](https://wiki.ubuntu.com/Debug%20Symbol%20Packages) is an official tutorial on how to install them.

## Further reading

If you're not satisfied with the basic CLI/TUI debugging options GDB offers, note that it is very easily extendable and scriptable.
You can write your own plugins or search through the web for existing ones.
For example the [GDB dashboard](https://github.com/cyrus-and/gdb-dashboard) offers a better TUI for debugging (variable watching, breakpoint list, stack etc.).
You can play around with the `~/.gdbinit` file and tweak a lot of the settings there to better suit your needs.

**GOOD HUNTING!**

[^1]: The `-O0` flag of `gcc`/`g++` disables optimizations, which makes debugging easier (although it potentially makes your program run slower). `-g` turns on generation of debugging symbols, which enables you to inspect code, print values of variables etc. during debugging.
[^2]: The `debug_roslaunch` script is a utility script, which will create a tmux split to separate output/output of your node and GDB in order to make debugging clearer. You can find it in the `uav_core` repository in case you don't have it.
