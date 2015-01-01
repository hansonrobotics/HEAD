# eva_behavior

Initial behaviour tree for Eva. Current verion designed to work with Dmitroid robot and current nodes already developed.

## Pre-requisites

The [Owyl Behavior Tree](https://github.com/eykd/owyl/) package is
required.  To install:

1) `git clone git@github.com:eykd/owyl.git`

2) `sudo python setup.py install`

## Co-requisites

This is meant to be used in conjuction with several ROS nodes:
* [robo_blender](http://github.com/hansonrobotics/robo_blender) which
  implements a movable robot head.
* [perception](http://github.com/hansonrobotics/perception) which
  organizes  visual data from cameras and other sensors.
* [pi_vision]((http://github.com/hansonrobotics/pi_vision) which
  provides visual data to the perception node.

## Running
To run, start the robo_blender, perception, pi_vision and uvc_cam nodes.
Then

`rosrun eva_behavior general_behavior.py`


## ROS Messaging API

#####Messages defined:

* `event`:
```
string event
string param
```
Specifies event(new_face,exit) with the face_id as parameter.

* `tracking_message`
```
string target
string action
string params
```
Specifies what to track (topic name that publishes
`sensor_msgs/RegionOFInterest` message), action (track, glance),
params (time to track).

##### Topics subscribed :

* `behavior_switch (std_msgs/String)`. Values: ("btree_on", "btree_off").
  By default the behaviour tree is off and it needs to receive
  btree_on to start.
* `tracking_event (eva_behavior/event)`. Event received from the
  perception nodes, currently only pi_vision.

##### Topics published

* `cmd_blendermode (std_msgs/String)`. Blender mode used for tracking:
  Dummy (do nothing), TrackDev(tracking objects), LookAround (looking
  for attention).
* `tracking_action (eva_behavior/tracking_message)`. Sends the command
  to blender for specific object tracking (track, glance).
* `make_coupled_face_expr (basic_head_api/MakeCoupledFaceExpr)`. Sends
  the expression and intensity for the current robot to basic_head_api node.


## Cookbook
Below follows a list of examples and demos.

### uhhh



