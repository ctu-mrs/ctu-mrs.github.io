---
layout: default
title: Suggested reading for newcomers
parent: Introduction
nav_order: 1
---

# Mandatory readings for newcomers

Nowadays, doing research in this field is not just about the theory and occasional lab experiment.
When we prepare our experiments, we rely on a relatively complex platform of software and utilities, which allows us to implement new techniques and test them in realistic conditions.
But knowing the fundamentals is only half of the success.
The following list is prepared to mitigate the initial friction between you and the platform.
Please follow it thoroughly and enjoy the learning!

## Mandatory Software to learn

### Ubuntu

All our software for drones runs on **Ubuntu 18.04/20.04 LTS, 64-bit**.
Please install this particular version; a lot of other software depends explicitly on it.
We strongly suggest installing a native system, not a virtual one.
Do not use Virtual Box, although it works fine, the drop in performance is noticeable, especially when simulating a real-time flying drone.

### Bash

Bash is the standard shell in Ubuntu.
Most of the time, when you work in the terminal, you use bash to run programs, for scripting and to manage your file system.

* Skills needed:
  * moving through a file system
  * writing and running simple shell scripts
  * grep, pipe \|, output redirection, echo, ssh, scp, rsync, cat, mounting devices (lsblk, mount, sync, umount)

### ROS - Robot Operating System

ROS is a middleware between Ubuntu and C++.
Thanks to it, our programs can talk to each other asynchronously.
It also allows simple control of your software from the terminal.
A lot of utilities for robotics have already been programmed with ROS, including sensor drivers, visualization, logging, etc.
Thus, we don't need to reinvent the wheel.

Getting into ROS is simple, follow tutorials on their webpage, and don't be afraid to experiment.
You can't break the drone in simulation :-).
Before trying the tutorials, please install our [UAV system](http://github.com/mrs_uav_system).
It will install all the tools we usually use.

* Tutorials: [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
  * It is not as important to know how to create repositories; we have already prepared the structure for you. But skimming through all the tutorials is recommended.
  * Suggested tutorials: 2, 5, and further.
* Required skills: The more, the better, most of it will come as you start working on your project.
* Our ROS examples:
  * If you feel like it, try to dig into our own ROS examples.
  * [Example ROS UAV](https://github.com/ctu-mrs/example_ros_uav) showcase most of our techniques.
  * [ROS example](https://github.com/ctu-mrs/example_ros_vision) is dedicated to online computer vision.

### GIT - code versioning system

GIT is a file versioning system.
All our code is stored and versioned using Git and hosted on [Gihub](https://github.com/ctu-mrs).
It allows collaborative work between many people and can be managed entirely from the terminal.

* Tutorials: [https://try.github.io/levels/1/challenges/1](https://try.github.io/levels/1/challenges/1), [https://git-scm.com/docs/gittutorial](https://git-scm.com/docs/gittutorial)
* Skills needed:
  * cloning repositories,
  * committing changes,
  * pushing and pulling changes to a server,
  * branching,
  * merging,
  * fixing merge conflicts (will come sooner or later).

Checkout out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.

### TMUX - terminal multiplexer

Tmux is a command-line utility that allows splitting a terminal to multiple panels and creating windows (tabs).
It is similar to, e.g., Terminator, but runs entirely in the command line.
Thus it can be used remotely over ssh.
It is scriptable, which makes it ideal for automating processes, where multiple programs are launches simultaneously.

* [https://github.com/tmux/tmux](https://github.com/tmux/tmux)
* We compile tmux 3.0a from sources.
* List of basic key bindings for our particular setup can be found here: [https://github.com/klaxalk/linux-setup/wiki/tmux](https://github.com/klaxalk/linux-setup/wiki/tmux)
* The key bindings should be familiar to those using Vim.

Checkout out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.

### Tmuxinator - automating tmux

Tmux itself is very powerful, tmuxinator is just adding some cream to it.
Tmuxinator uses .xml files containing a description of a tmux session.
It allows us to define and automate complex multi-terminal setups for, e.g., development (one session per program) and simulations.
All our simulation startup script are written for tmuxinator.

* [https://github.com/tmuxinator/tmuxinator](https://github.com/tmuxinator/tmuxinator)

### Vim (~~recommended~~ mandatory)

Everyone should use a tool that is right for the job.
Well, for our purposes (C++, ROS, Python, bash), Vim is very well suited.
A lot of the time, you will find yourself in need of editing a code remotely (over ssh), and Vim can provide IDE-like features even in that situation.
You might hear that Vim is "tough to learn" or "Vim is for crazy people," but in reality, it is not valid.
Vim provides an efficient way to edit (explicitly written edit, because editing is what you mostly do while programming) which will pay itself off after you learn it.
Learning Vim is about changing the paradigm of editing - it's more about controlling a machine (synthesizer) that edits text, rather than moving a cursor with a mouse and then typing.
Working in the terminal, using, e.g., **tmux** and ** Vim** can also help you put away your mouse.
Yes, a mouse is not an ideal tool for programming, though it has its use in gaming, 3D modeling, video editing, and so on.
Without a mouse, you will become much more productive, and your carpal tunnels will thank you.

* Our programming and simulation environment (a.k.a, the [Linux Setup](https://github.com/klaxalk/linux-setup)) is distributed with Vim already containing a lot of useful features including but not limited to **fully functional code completion** working with ROS, code snippets, ROS integration, Tmux integration, Latex development, etc.
* Don't be afraid to ask about Vim, we will gladly help.
* Give it a chance, have a look at following videos:
  * [https://www.youtube.com/watch?v=_NUO4JEtkDw](https://www.youtube.com/watch?v=_NUO4JEtkDw)
  * [https://www.youtube.com/watch?v=5r6yzFEXajQ](https://www.youtube.com/watch?v=5r6yzFEXajQ)
* A list of particular features and key bindings used in our setup can be found here: [http://github.com/klaxalk/linux-setup/wiki](http://github.com/klaxalk/linux-setup/wiki)

Checkout out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.

You make it down here! Great! Leave us a message on what we could improve about these recommendations
