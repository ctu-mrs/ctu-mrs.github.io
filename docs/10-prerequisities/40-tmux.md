---
title: Tmux
pagination_label: Tmux terminal multiplexer
description: Tmux terminal multiplexer
---

# Tmux terminal multiplexer

Tmux is a command-line utility that allows splitting a terminal to multiple panels and creating windows (tabs).
It is similar to, e.g., Terminator, but runs entirely in the command line.
Thus it can be used remotely over ssh.
It is scriptable, which makes it ideal for automating processes, where multiple programs are launches simultaneously.

* [https://github.com/tmux/tmux](https://github.com/tmux/tmux)
* We compile tmux 3.0a from sources.
* List of basic key bindings for our particular setup can be found here: [https://github.com/klaxalk/linux-setup/wiki/tmux](https://github.com/klaxalk/linux-setup/wiki/tmux)
* The key bindings should be familiar to those using Vim.

Checkout out our [Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) for some useful tips.

## Tmuxinator

Tmux itself is very powerful, tmuxinator is just adding some cream to it.
Tmuxinator uses .xml files containing a description of a tmux session.
It allows us to define and automate complex multi-terminal setups for, e.g., development (one session per program) and simulations.
All our simulation startup script are written for tmuxinator.

* [https://github.com/tmuxinator/tmuxinator](https://github.com/tmuxinator/tmuxinator)
