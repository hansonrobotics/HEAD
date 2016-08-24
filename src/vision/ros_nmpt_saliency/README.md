<<<<<<< HEAD
### Visual Saliency Detection using NMPT
=======
### Visual Salience Feature Perception using NMPT
>>>>>>> 9789e363033dbffea6e22efa9458689913a22743
------------

###Introduction
This package listens to `/cv_camera/image_raw` topic and analyzes the video for saliency. It gives a point output `0..1` for x, `0..1` for y, and the degree of salient point ranging from `1..10`. The messages are pulblished to the topic  `/nmpt_saliency_point`. </br>
<<<<<<< HEAD

1. Package Name: `ros_nmpt_saliency`
2. Node Name:    `ros_nmpt_saliency_node`
3. Topics:       `/nmpt_saliency_point`
4. MessageFiles: `targets.msg` </br>
   4.1 Type 1: `geometry_msgs/Point[] positions` </br>
   4.2 Type 2: `float32 degree` 

###How it works
####Salient Point Detector
TBW
####Degree
=======
Besides that, it contains a node that lets Sophia track salient features in the region of interest based on the state in "/camera/face_event" and Degree of certainity of identified Salient Point </br>

1. Package Name: `ros_nmpt_saliency`
2. Subscribes for: `/camera/face_event` and `/camera/face_locations`
3. Publishes to: `/nmpt_saliency_point`
4. MessageFile: `targets.msg`  that subsumes data which have `geometry_msgs/Point[]`  and `float32` type </br>

###How it works
####Salient Point Detector
This node is composed of two features, namely, edge detection and movement detection. The edge detection component returns the attributes of points addressed by the edge. On the other hand, the movement detector gets the smoothed absolute difference of the current frame and the frame prior to it. Points from the resultant edges are sent to the salience detector which further decides on the saliency of a point relative to other points.

####Degree of Salience
The degree calculator accepts three coordinates (x,y, and z) as coordinates of salient point. Given N second turnaround time it calculates the rate of change of a specific salient point. The best possible interval of change for a given salient point (t) is used to normalize degrees of salient points overtime. i.e the maximum interval before change <-> the  minmum increment on the degree of the given salient point. Furthermore, the degree of each salient point is normalized in range 1 to 10.

####Salient Feature Tracking
`salient_feature_tracker` is a node that lets the robot track salient points. This node becomes active when there is no face in the region of interest. NMPT based salience detector is sensetive to movement and could be used to track Person movement. Specifically, Hand, and Face movements are most frequently observable changes. Hence, this node could also help to facilitate Face Detection. 
Currently, the code is managed to track salient points with High Degree of certainity (>=5).</br>

###Get the Repo

[HEAD](https://github.com/hansonrobotics/HEAD/blob/master/README.md) </br>

###Running

`cd hansonrobotics && ./scripts/dev.sh` </br>

###Subscription to the Topic
1. Check `salient_feature_tracker` node as a reference. </br> 

###Troubleshooting
[Issues](https://github.com/hansonrobotics/HEAD/issues) </br>

### Feature Work
* OpenCog Integration and writing OpenPsi based Rule
* Distance of ROI: currently, the distance for look/gaze at is set to 1 but must have a way to get the real distance of ROI.
>>>>>>> 9789e363033dbffea6e22efa9458689913a22743

The degree calculator accepts three coordinates (x,y, and z) as coordinates of salient point. Given N second turnaround time it calculates the rate of change of a specific salient point. The mimumum possible interval of change for a given salient point (t) is used to normalize degrees of salient points overtime. i.e the maximum interval before change <-> the  minmum increment of the degree of salient point. Furthermore, the degree of each salient point is normalized in range 1 to 10.

###Get the Repo

[HEAD](https://github.com/hansonrobotics/HEAD/blob/master/README.md) </br>

###Running

`cd hansonrobotics && ./scripts/dev.sh` </br>

###Halt the Execution
`cd hansonrobotics && ./scripts/stop.sh` </br>

###Subscription to the Topic
1. Check `sample_nmpt_subscriber` node as a reference. </br> 

###Troubleshooting

  TBW

###FAQ

  TBW
</br>

###Auto-Test
  TBW
###Reference
----------------
 TBW

