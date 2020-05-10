---
layout: default
title: Suggested reading for newcomers
parent: Introduction
---

# Mandatory readings for newcomers

Nowadays, doing science is not just about the math and occasional code writing.
When we prepare our experiments, we rely on a relatively complex platform of software and utilities, which allows us to implement new techniques and test them easily.
Learning the technology is half of the success.
The following list is prepared to mitigate the initial friction between you and the software in mind.
Please follow it thoroughly and enjoy the learning!

## Mandatory programs to learn about

### Ubuntu

All our software for drones runs on Ubuntu 18.04 LTS 64-bit.
Please install this particular version, a lot of other software specifically depends on it.
We strongly suggest to install a native system, don't virtualize.
Do not use Virtual Box, although it works fine, the drop in performance is really noticeable, especially when simulating a real-time flying drone.

### Bash

Bash is the standard shell in Ubuntu.
Most of the time, when you work in the terminal, you use bash to run programs, for scripting and to manage your file system.

  * Skills needed:
    * moving through a file system
    * writing and running simple shell scripts
    * grep, using |, output redirection, echo, ssh, scp, rsync, cat, mounting devices (lsblk, mount, sync, umount)

### ROS - Robot Operating System

ROS is a middleware between Ubuntu and C++.
Thanks to it, our programs can talk to each other asynchronously, even though they don't have to "know" each other necessarily.
It also allows simple control of your software from the terminal.
A lot of robotic stuff has been already programmed in ROS, including sensor drivers, visualization, planning, etc., thus we don't need to reinvent the wheel.

Getting into ROS is simple, just follow tutorials on their webpage and don't be afraid to experiment. You can't break the drone in simulation :-).
Prior to trying the tutorials, please install our simulation software.

  * Tutorials: http://wiki.ros.org/ROS/Tutorials
    * It is not as important to know how to create repositories; we have already prepared the structure for you. But skimming through all the tutorials is recommended.
    * Suggested tutorials: 2, 5, and further.
  * Required skills: The more, the better, most of it will come as your start working on your project.

### GIT - code versioning system

GIT is a powerful versioning system.
All our code is stored and versioned using GIT.
It allows collaborative work between many people and can be managed completely from the terminal.
Our repositories are hosted on http://mrs.felk.cvut.cz/gitlab or http://github.com.

  * Tutorials: https://try.github.io/levels/1/challenges/1 https://git-scm.com/docs/gittutorial
  * Skills needed:
    * cloning repositories
    * committing changes
    * pushing and pulling changes to a server
    * branching
    * merging
    * fixing conflicts (will come as they appear :-))

### TMUX - terminal multiplexer

Tmux is a command line utility that allows splitting a terminal to multiple panels and creating windows (tabs).
It is similar to e.g. Terminator, but runs completely in the command line.
Thus it can be used over ssh.
It is scriptable, which makes it ideal for automating simulations, where multiple programs are running in parallel.

  * https://github.com/tmux/tmux
  * We compile tmux 2.2 from sources while installing our development environment.
  * List of basic key bindings for our particular setup can be found here: https://github.com/klaxalk/linux-setup/wiki/tmux 
  * The key bindings should be familiar to those using vim.

### Tmuxinator - automating tmux

Tmux itself is very powerful, tmuxinator is just adding some cream on it.
Tmuxinator uses .xml files containing the description of a tmux session.
It allows to define and automate complex multi-terminal setups for e.g. development (one session per program) and simulations.

  * https://github.com/tmuxinator/tmuxinator

### Vim (~~recommended~~ mandatory)

Everyone should use a tool that is right for the job.
Well, for our purposes (C++, ROS, python, bash), Vim is very well suited.
A lot of the time, you will find yourself in need of editing a code remotely (over ssh), and Vim can provide IDE-like features even in that situation.
You might hear that Vim is "tough to learn" or "Vim is for crazy people," but in reality, it is not true.
Vim provides an efficient way to edit (I explicitly say edit, because editing is what you mostly do during programming) which will pay itself off after you learn it.
Learning Vim is about changing the paradigm of programming - it's more about controlling a machine (synthesizer) that edits a text, rather than moving a cursor with a mouse and then typing the text.
Working in the terminal, using e.g. **tmux** and **vim** can also help you put away your mouse.
Yes, a mouse is not an ideal tool for programming, though it has its use in gaming, 3D modeling, video editing and so on.
Without a mouse, you will become much more productive, and your carpal tunnels will thank you.

  * Our programming and simulation environment is distributed with Vim already containing a lot of useful features including but not limited to **fully functional code completion** working with ROS, code snippets, ROS integration, Tmux integration, Latex development, etc.
  * Don't be afraid to ask about Vim, we will gladly help.
  * Give it a chance, have a look at following videos: 
    * https://www.youtube.com/watch?v=_NUO4JEtkDw
    * https://www.youtube.com/watch?v=5r6yzFEXajQ
  * A list of particular features and key bindings used in our setup can be found here: http://github.com/klaxalk/linux-setup/wiki
