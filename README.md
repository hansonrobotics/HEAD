blender_api_msgs
================

ROS interface controlling the head+body `blender_api` rig.

# Design goals
* Avoids adding ROS code to the blender_api python module
* Eliminates the need for other ROS nodes from having to import
  the blender_api python module.

# Installation
```
$ cd your_catkin_workspace
$ git clone https://github.com/hansonrobotics/blender_api_msgs.git
$ pip3 install ./blender_api_msgs
$ cd ../
$ catkin_make
```

# Usage
Run `blender_api` with the **autostart script** or press the **Start Command
Listener** button.

If this package is installed with pip or setuptools, `blender_api` should
discover it automatically.
