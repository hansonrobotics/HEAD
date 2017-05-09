icog_face_tracker
================

A ROS package for face detection using OpenCV and OpenTLD.

Prerequisites
-------------
###### usb_cam
> A ROS Driver for V4L USB Cameras
> http://wiki.ros.org/usb_cam | apt-get install ros-indigo-usb-cam

###### opencv
> Opensource computer vision library.
> http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html

###### CV Bridge
> http://wiki.ros.org/cv_bridge | apt-get install ros-indigo-cv-bridge

###### Image Transport
> http://wiki.ros.org/image_transport | apt-get install ros-indigo-image-transport

Build
-----
```sh
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
$ cd src
$ git clone https://github.com/Selameab/icog_face_tracker.git
$ cd ..
$ catkin_make
```

Run
---
```sh
roslaunch icog_face_tracker tracker_tld.launch
```

ROS Nodes
---------
#### /tracker_tld_node

###### Publications: 
 * /faces [icog_face_tracker/faces]
 
###### Subscriptions: 
 * /usb_cam_node/image_raw [sensor_msgs/Image]
 

#### /usb_cam_node

###### Publications: 
 * /usb_cam_node/image_raw [sensor_msgs/Image]