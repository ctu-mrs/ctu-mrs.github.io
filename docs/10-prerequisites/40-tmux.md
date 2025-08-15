---
title: Tmux
pagination_label: Tmux terminal multiplexer
description: Tmux terminal multiplexer
---

# Tmux terminal multiplexer

[Tmux](https://github.com/tmux/tmux) is a command-line utility that allows splitting a terminal to multiple panels and creating windows (tabs).
It is similar to, e.g., Terminator, but runs entirely in the command line.
Thus it can be used remotely over ssh.
It is scriptable, which makes it ideal for automating processes, where multiple programs are launches simultaneously.

We utilize tmux for deployment of the MRS system when the system is *installed natively*.

* List of basic key bindings for our particular setup can be found here: [https://github.com/ctu-mrs/mrs_uav_shell_additions/blob/master/package/etc/ctu-mrs/tmux.conf](https://github.com/ctu-mrs/mrs_uav_shell_additions/blob/master/package/etc/ctu-mrs/tmux.conf)
* The key bindings should be familiar to those using Vim.

Check out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.

## Tmuxinator

Tmux itself is very powerful, tmuxinator is just adding some cream to it.
Tmuxinator uses .xml files containing a description of a tmux session.
It allows us to define and automate complex multi-terminal setups for, e.g., development (one session per program) and simulations.
All our simulation startup script are written for tmuxinator.

* [https://github.com/tmuxinator/tmuxinator](https://github.com/tmuxinator/tmuxinator)
