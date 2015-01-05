pi_vision
=========

Pi Vision ported to ROS Indigo, together with various enhancements:
* Its been catkinized.
* Can track multiple faces.
* Attempts to perform localization in 3D, so that face positons are now
  published in 3D coordinates.

This is based on the original Pi Vision package, taken from
http://wiki.ros.org/pi_vision, which had been abandonded after groovy.
This package includes both includes http://wiki.ros.org/pi_face_tracker
and http://wiki.ros.org/pi_face_tracker_gui

The plain, unenhanced  port of pi_vision to indigo can be found in the
branch `pi-vision-orig-indigo`.


Installation
============
Requirements:
* ros-indigo-openni-camera
* mjpeg_server
* usb_cam


```
apt-get install ros-indigo-mjpeg-server

apt-get install ros-indigo-openni-camera

apt-get install ros-indigo-usb-cam

```

The node must be built from git:

```

git clone https://github.com/hansonrobotics/pi_vision
cd catkin; catkin build
source devel/setup.bash
```

Run
===

To change camera settings modify the usb_cam.launch file.

```
roslaunch ros2opencv usb_cam.launch 
roslaunch pi_face_tracker face_tracker_usb_cam.launch
```

ROS Nodes
=========

## /face_locations
Publishes a list of human faces being tracked. Each is given an ID number,
and a 3D coordinate.  The coordinate frame used is the usual ROS
'engineering' frame: `x` is straight ahead, `y` is the the left, and `z`
is up.  Units are in meters.

## /face_event
Publishes face tracking events. Currently, the only events published are
`new_face` and `lost_face`. The first indicates a newly-acquired face to
track, the second, that a face is no longer visible.

TODO
====
1. Make it work with Kinect.
2. Migrate to Python cv2 wrappers from cv.  The cv wrappers use a
   c++-like interface, the cv2 wrappers use numpy and are more efficient.
