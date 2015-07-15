# eva_behavior

Initial behaviour tree for Eva. Current verion works with the
[Eva blender head](http://github.com/hansonrobotics/blender_api)
and should also continue to work with the older, less sophisticated
[Arthur robot](http://github.com/hansonrobotics/robo_blender) head.

## Pre-requisites

The [Owyl Behavior Tree](https://github.com/eykd/owyl/) package is
required.  To install:

1) `git clone git@github.com:eykd/owyl.git`

2) `sudo python setup.py install`

## Co-requisites

This is meant to be used in conjuction with several ROS nodes:
* [blender_api](http://github.com/hansonrobotics/blender_api)
  implements a model of the Eva head in blender.
* [blender_api_msgs](http://github.com/hansonrobotics/blender_api_msgs)
  defines ROS messages to control the blender model.
* [perception]((http://github.com/hansonrobotics/perception) which
  publishes 3D coordinates for interesting things (people's faces) for
  the hed to look at.
* [pi_vision]((http://github.com/hansonrobotics/pi_vision) which
  provides a face detector, so that the robot can 'see' humans in
  its visual field.

Please review the above for install instructions.  Don't forget to:
* catkin build
* source devel/setup.bash

Alternately, use the [Eva Dockerfile](https://github.com/opencog/docker)
to build a pre-configured docker container containing all of he needed
packages.

## Running
To run, start the blender_api, perception, pi_vision and uvc_cam nodes.
Then
```
rosrun eva_behavior main.py
```
Then, turn on behaviors:
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on
```
There is a second mode, the so-called "stage mode", where all gestures
are exaggerated, thus making them more easily visible if the audience
is farther away, and don't have a good view of the head.  Note that, up
close, the "stage mode" will look quite manic.
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on_stage
```
Both commands will cause the system to play through the set of scripted
behaviors, which includes exprsssing a variety of emotions,
looking at the various people (faces) as they become visible to the
robot camera, and so on.

The relative scaling of the gestures, when running close-up vs. stage
mode, is controlled in the `behavior.cfg` file, by the following
factors:
```
emotion_scale_stage
emotion_scale_closeup
gesture_scale_stage
gesture_scale_closeup
```

Other behavior is configued in this same file.

## ROS Messaging API

##### Topics subscribed :

* `behavior_switch (std_msgs/String)`. Values: (`btree_on`,
  `btree_on_stage`, `btree_off`, `emotion_off`, `gestue_off`).
  By default, the behaviour tree is off and it needs to receive
  `btree_on` or `btree_on_stage` (for stage mode) to start.

  * `btree_on`: Facial expressions are scaled for up-close interaction;
    robot and human approx 1 meter apart.
  * `btree_on_stage`: Facial expressions are scaled for a distant
    audience; multiple spectators are more than 3 meters away.
  * `emotion_off`: disable publication of ROS emotion messages.
  * `gesture_off`: disable publication of ROS gesture messages.

  The `btree_on` and `btree_on_stage` messages re-enable emotion and
  gesture publication.

* `tracking_event (eva_behavior/event)`. Event received from the
  perception nodes, currently only pi_vision.

##### Topics published

* `/blender_api/set_face_target (blender_api_msg/Target)`. Sends
  location that the robot head should face and look at.

* `/blender_api/set_gaze_target (blender_api_msg/Target)`. Sends
  location that the robot eyes should look at (without moving the
  head).


## Cookbook
Below follows a list of examples and demos.

### Basic random emotion cycling
Start up one of the blender heads, then:
```
rosrun eva_behavior main.py
```
Turn on behaviors:
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on
```
OR in this way (for stage mode):
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on_stage
```
The head should now be cycling through a set of facial expressions.

### Face Tracking
As above, but also start uvc_cam and pi_vision. Note that uvc_cam
won't automatically show up in the `ROS_PACKAGE_PATH` environment
variable; it must be added by hand.
```
export ROS_PACKAGE_PATH=/path/to/uvc_cam:$ROS_PACKAGE_PATH
roslaunch ros2opencv uvc_cam.launch device:=/dev/video1
roslaunch pi_face_tracker face_tracker_uvc_cam.launch
```
