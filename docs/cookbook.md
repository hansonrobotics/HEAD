
Cookbook
========
A collection of recipies and HOWTO instructions for using this ROS node.

Before running, be sure to do the following steps:
```
$ source /opt/ros/indigo/setup.bash
$ cd your_catkin_ws
$ git clone https://github.com/hansonrobotics/blender_api_msgs.git
$ catkin build
$ source devel/setup.bash
```
The `blender_api_msgs` are used to define the messages that this node
uses.

##API Version
In order to maintain compatibility, the node publishes a ROS version number:
```
$ rostopic echo -n 1 /blender_api/get_api_version
```
which should currently return `1`.


##Get Animation List
Obtain the list of supported animations.
```
rostopic list
rostopic echo -n 1 /blender_api/available_emotion_gestures
rostopic echo -n 1 /blender_api/available_emotion_states
```
These should display the following outputs:
```
$ rostopic echo -n 1 /blender_api/available_emotion_gestures
data: ['blink', 'blink-micro', 'blink-relaxed', 'blink-sleepy', 'nod-1', 'nod-2', 'nod-3', 'shake-2', 'shake-3', 'yawn-1']
---
linas@fanny: ~/src/novamente/hanson/src/hr/blender_api $ rostopic echo -n 1
/blender_api/available_emotion_states
data: ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused', 'afraid', 'bored', 'engaged', 'amused', 'comprehending']
---
```

## Running Animations
Uhh
```
rostopic pub --once /blender_api/set_emotion_gesture std_msgs/String nod-1
```
and um ... 

duration can only be an int, or the list [secs, nanosecs]
```
rostopic pub --once /blender_api/set_emotion_state blender_api_msgs/EmotionState '{name: sad, magnitude: 0.5, duration: [2, 500000000]}'
```
