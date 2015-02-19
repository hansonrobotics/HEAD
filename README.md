Perception Synthesizer
======================

Planned function: accept video (pi_vision) from multple sources, convert
to 3D spatial coordinates using ros-tf.  Publish intersting events to all
concerned. Interesting events include:

* Entry, exit of a human face in the field of view.
* Location of the face.

There are several goals to be acheived:

* Integrate multiple camera inputs. In particular, the eye cameras move
  with the eyes, and thus have a constantly changing field-of-view,
  whereas the body camera has an essentially fixed field-of-view.  The
  information from these sources needs to be integrated.

* Behaviour nodes that need perception information should be able to
  to poll for it on an as-needed basis, rather than being driven by
  a 20-frames-per-second perception updates.


##Prerequisites
pi_vision is needed. See
[pi_vision](https://github.com/hansonrobotics/pi_vision) for details.

## Running, Testing
There are several ways to get things running. One way is to manually
start the usb camera node, and the pi_vision node, and then the
perception node.  A simpler alternative is to use the provided launch
files to do this automatically. So:

### Single (body) camera startup
Just run the launch files.  You may want to adjust single-cam.launch
to match your actual webcam.
```
roslaunch perception tracker-single-cam.launch
roslaunch perception geometry.launch
```
### Two-camera (body+eye) startup
```
roslaunch perception tracker.launch
roslaunch perception geometry.launch
```

### Disable rviz visualization
The rviz visualizer can be disabled by saying:
```
roslaunch perception geometry.launch gui=false
```

### Alternative URDF files:
Alternative URDF files can be specified by saying
```
roslaunch perception geometry.launch model:=other.urdf
```
to use the `other.urdf` file.

### Manual startup
 * Make sure the usb camera is running, typically like this:
```
roslaunch ros2opencv usb_cam.launch
```
 * Make sure pi_vision is running:
```
roslaunch pi_face_tracker face_tracker_usb_cam.launch
```
 * Make sure that the face tracker is publishing faces:
```
rostopic echo /camera/face_locations
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
 * `camera/face_locations (pi_vision/Faces)`: Faces published by
   pi_vision, as seen by main camera.
 * `eye_camera/face_locations (pi_vision/Faces)`:  Faces published by
   pi_vision, as seen by eye camera.

###### Params topics
 * `max_distance`: Max distance for faces from eye-camera and body
   camera to be considered same.

###### Transformations published:
 * `face_base<face_id>`: Face location based on body camera
 * `Face<face_id>`: Transformation relative to the `face_base` from eye
   camera.

###### Testing
 * Make sure you have two cameras connected:
   - Body Camera on `/dev/video2`
   - Eye Camera on `/dev/video1`
 * Run face tracking: `roslaunch perception tracker.launch`
 * Run the node: `roslaunch perception display.launch`
