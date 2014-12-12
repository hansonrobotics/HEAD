blender_api_msgs
================

ROS messages defining the interface controlling the head+body blender
rig. Specifically, the ROS node will invoke python calls to the
abstract python class 'blender API'. Different blender rigs use this
common class to implement animations.


# Design goals
* Avoids adding ROS code to the blender_api python module
* Eliminates the need for other ROS nodes from having to import
  the blender_api python module.

# Usage
TBD
