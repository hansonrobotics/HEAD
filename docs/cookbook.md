
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
rostopic echo -n 1 /blender_api/available_gestures
rostopic echo -n 1 /blender_api/available_emotion_states
```
These should display the following outputs:
```
$ rostopic echo -n 1 /blender_api/available_gestures
data: ['blink', 'blink-micro', 'blink-relaxed', 'blink-sleepy', 'nod-1', 'nod-2', 'nod-3', 'shake-2', 'shake-3', 'yawn-1']
---
linas@fanny: ~/src/novamente/hanson/src/hr/blender_api $ rostopic echo -n 1
/blender_api/available_emotion_states
data: ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused', 'afraid', 'bored', 'engaged', 'amused', 'comprehending']
---
```

## Running Animations
Individual gestures can be launched one at a time. For example, a
single, short nod:
```
rostopic pub --once /blender_api/set_gesture std_msgs/String nod-1
```

Emotional states require a magnitude and duration to be specified, and
so the message format is more complex. The duration can be a single
integer, which is interpreted as seconds, or a list of two integers
which is interpreted as [seconds, nanoseconds].  Thus, below, the list
[6, 500000000] represents six-and-a-half seconds.

```
rostopic pub --once /blender_api/set_emotion_state blender_api_msgs/EmotionState '{name: sad, magnitude: 1.0, duration: [6, 500000000]}'
```
Multiple emotions can be specified in rapid succession; these will be
blended together.

##Monitoring Emotional State
The current emotional state is continually published, and can be
monitored:
```
rospic echo /blender_api/get_emotion_states
```
A neutral emotional state is reported as an empty list:
```
data: []
---
data: []
---
```
while an emotion that is active might appear as:
```
---
data:
  -
    name: happy
    magnitude: 0.328999996185
    duration:
      secs: -402
      nsecs: 903000001
---
```
Multiple blended emotions will be reported as such:
```
---
data:
  -
    name: afraid
    magnitude: 0.757000029087
    duration:
      secs: -43
      nsecs: 133000001
  -
    name: happy
    magnitude: 0.202999994159
    duration:
      secs: -794
      nsecs: 491000000
---
```


The gesture state can also be monitored.  Under normal circumstances,
the rig is animated so as to breath, and thus will publish a continuous
stream of messages. Thus,
```
rostopic echo /blender_api/get_gestures
```
will show
```
---
data:
  -
    name: CYC-normal
    magnitude: 1.0
    duration:
      secs: 1108
      nsecs: 944999999
    speed: 1.0
  -
    name: CYC-breathing
    magnitude: 0.5
    duration:
      secs: 1275
      nsecs: 878999999
    speed: 1.0
---
```
Specific gestures will have the GST prefix, and so
```
data:
  -
    name: GST-nod-3
    magnitude: 0.5
    duration:
      secs: 41
      nsecs: 399999999
    speed: 1.0
  -
    name: CYC-normal
    magnitude: 1.0
    duration:
      secs: 507
      nsecs: 5999999
    speed: 1.0
  -
    name: CYC-breathing
    magnitude: 0.5
    duration:
      secs: 717
      nsecs: 332999999
    speed: 1.0
---
```
will appear after a nod request:

```
rostopic pub --once /blender_api/set_gesture std_msgs/String nod-3
```
