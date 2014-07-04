pi_vision
=========

Pi Vision ported to ROS Indigo.

Installation
============

```
git clone https://github.com/ericperko/uvc_cam.git
rosmake uvc_cam


apt-get install ros-indigo-openni-camera
# If error occureed sub-process /usr/bin/dpkg returned an error code (1)
#dpkg --remove --force-remove-reinstreq libopenni-sensor-pointclouds0
#apt-get install -f -y

apt-get install ros-indigo-mjpeg-server

git clone https://github.com/hansonrobotics/pi_vision
rosmake pi_vision
```

Run
=======
```
roslaunch ros2opencv uvc_cam.launch device:=/dev/video1
roslaunch pi_face_tracker face_tracker_uvc_cam.launch
```


TODO
============
1. Make it work with Kinect
2. Migrate to Python cv2 from cv

