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
 * Make sure pi_vision is running, and that the face tracker is
   publishing faces:
```
rostopic echo /face_locations
```
 * Start the perception node:
```
rosrun perception faces_tf2_broadcaster.py
```
 * Verify that the 3D location of the face is reasonable; launch rviz to
   see where it is.
```
rosrun rviz rviz -d `rospack find perception`/rviz/faces.rviz
```

## ROS Nodes
### faces_tf2_broadcaster
Broadcast `/tf` data from face tracking.

###### Subscribed topics
 * `camera/face_locations (pi_vision/Faces)`: Faces published by pi_vision in 3D coordinates by main camera
 * `eye_camera/face_locations (pi_vision/Faces)`:  Faces published by pi_vision in 3D coordinates by eye camera
 
###### Params topics
 * `max_distance`: Max distance for faces from eye-camera and body camera to be considered same.

###### Transformations published:
 * `face_base<face_id>`: Face location based on body camera
 * `Face<face_id>`: Transformation relative to the `face_base` from eye camera.

###### Testing
 * Make sure you have two cameras connected:
   - Body Camera on `/dev/video0`
   - Eye Camera on `/dev/video1`
 * Run face tracking: `roslaunch perception tracking.launch`
 * Run the node: `roslaunch perception display.launch` 
