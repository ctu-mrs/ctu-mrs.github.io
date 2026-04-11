---
title: FAQ
---

# Frequently Asked Questions
**I still see old config/launch.py (yaml/py) files even after I edited them in my packages?**
In ROS2, the config (yaml) and launch.py (python) files are copied to the `install` directory of your workspace during the install phase of your build process. You can verify this by looking at the `install()` functions inside the `CmakeLists.txt` file of your package. So, if you edit the config (yaml) or launch.py (python) files, you need to run the `colcon build` command to copy the modified files to the `install` directory. 
