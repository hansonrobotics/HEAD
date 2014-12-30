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
* uvc_cam

The first two can be obtained via `apt`:

```
apt-get install ros-indigo-mjpeg-server

apt-get install ros-indigo-openni-camera
# If error occureed sub-process /usr/bin/dpkg returned an error code (1)
#dpkg --remove --force-remove-reinstreq libopenni-sensor-pointclouds0
#apt-get install -f -y
```

The rest must be built from git:
```
git clone https://github.com/ericperko/uvc_cam.git
rosmake uvc_cam

git clone https://github.com/hansonrobotics/pi_vision
cd catkin; catkin build
source devel/setup.bash
```

Run
===

Note that uvc_cam won't automatically show up in the `ROS_PACKAGE_PATH`
environment variable; it must be added by hand.  Otherwise, the launch
will fail.
```
export ROS_PACKAGE_PATH=/path/to/uvc_cam:$ROS_PACKAGE_PATH
roslaunch ros2opencv uvc_cam.launch device:=/dev/video1
roslaunch pi_face_tracker face_tracker_uvc_cam.launch
```


TODO
====
1. Make it work with Kinect
2. Migrate to Python cv2 from cv

