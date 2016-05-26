ros_nmpt_saliency
=================

This package listens to /cv_camera/image_raw topic and analyzes the video for saliency. It gives a point output 0..1 for x, 0..1 for y to specify where in the image the saliency point lies by publishing it on /nmpt_saliency_point.

Usage
-----

rosrun ros_nmpt_saliency ros_nmpt_saliency_node


