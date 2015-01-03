Perception Synthesizer
======================

Planed function: accept video (pi_vision) from multple sources, convert
to 3D spatial coordinates using ros-tf. Publish intersting events to all
concerned. Interesting events include:

* Entry, exit of a human face in the field of view.
* Location of the face.

The goal here is to make sure that the behavior nodes have access to this
information, rather than having it drive the facial responses directly.

##Prerequisites
pi_vision is needed. See
[pi_vision](https://github.com/hansonrobotics/pi_vision) for install &
run details.

## Running, Testing
 * Make sure pi_vision is running.
```
rosrun image_view image_view image:=/pi_face_tracker/image
```
 * Start the perception node:
```
rosrun perception faces_tf2_broadcaster.py
```
 * Make sure the face tracker is publishing faces
```
rostopic echo /faces3d
```
 * Verify that the 3D location of teh face is reasonable; launch rviz to
   see where it is.
```
rosrun rviz rviz -d `rospack find perception`/rviz/faces.rviz
```

## ROS Nodes
### faces_tf2_broadcaster
Broadcast `/tf` data from face tracking.

##### Subscribed topics
 * `/faces3d (pi_vision/Faces)`: Faces published by pi_vision in 3D coordinates

