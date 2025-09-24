---
title: Python
pagination_label: Python
description: Python
---

# Python

Python is generally recommended as the programming language for quick prototyping, and developing software that does not need to be executed quickly/frequently. This may include mission progress tracking, waypoint planning, analyzing flight recordings etc. Components which rely on fast execution (state estimators, flight controllers, onboard data processors...) should be written in [C++](https://ctu-mrs.github.io/docs/software/cpp).

Coding with Python has its perks, mainly:
* no need to re-compile after every change
* simpler syntax than C++
* plenty of built-in libraries

but it also comes with its drawbacks:
* no compilation means errors only occur in runtime - not ideal when runtime means flying robots!
* generally slower runtime performance than C++
* versions change often, libraries change often, backwards compatibility not guaranteed

### Python2 vs Python3
There are currently 2 main versions of Python being used: Python2 and Python3. Python2 is no longer being developed, but remains in use in some areas for various reasons. We **strongly recommend using Python3**, which is actively maintained and developed.

### General resources

* The official online documentation for Python offers plenty of tutorials for beginners as well as in-depth look into the language: [https://docs.python.org/3/tutorial/introduction.html](https://docs.python.org/3/tutorial/introduction.html)
* [CZECH ONLY]: Dr. Petr Štěpán - [Krátká výuková videa](https://cw.fel.cvut.cz/wiki/courses/b3b33alp/cviceni/kratka_videa/start)

### First steps with ROS in Python

A simple guide on how to write your first ROS node in Python is available here: [https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).
