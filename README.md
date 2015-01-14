# eva_behavior

Initial behaviour tree for Eva. Current verion designed to work with
Dmitroid robot and current nodes already developed.

## Pre-requisites

The [Owyl Behavior Tree](https://github.com/eykd/owyl/) package is
required.  To install:

1) `git clone git@github.com:eykd/owyl.git`

2) `sudo python setup.py install`

## Co-requisites

This is meant to be used in conjuction with several ROS nodes:
* [blender_api_msgs](http://github.com/hansonrobotics/blender_api_msgs)
  defines the robot head controlling messages.
* [robo_blender](http://github.com/hansonrobotics/robo_blender) which
  implements a movable robot head.
* [pi_vision]((http://github.com/hansonrobotics/pi_vision) which
  provides visual data to the perception node.

Don't forget to:
* catkin build
* source devel/setup.bash

## Running
To run, start the robo_blender, perception, pi_vision and uvc_cam nodes.
Then
```
rosrun eva_behavior main.py
```
Then, turn on behaviors:
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on
```
OR in this way (for stage mode):
```
rostopic  pub --once /behavior_switch std_msgs/String btree_on_stage
```
This will cause the system to play through the set of scripted
behaviors, which includes exprsssing a variety of emotions,
looking at the various faces as they become visible, and so on.


## ROS Messaging API

##### Topics subscribed :

* `behavior_switch (std_msgs/String)`. Values: ("btree_on", "btree_on_stage", "btree_off").
  By default the behaviour tree is off and it needs to receive
  btree_on or btree_on_stage (for stage mode) to start.
* `tracking_event (eva_behavior/event)`. Event received from the
  perception nodes, currently only pi_vision.

##### Topics published

* `cmd_blendermode (std_msgs/String)`. Blender mode used for tracking:
  Dummy (do nothing), TrackDev(tracking objects), LookAround (looking
  for attention).

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
