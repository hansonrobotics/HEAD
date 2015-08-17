# Robot configuration files

Robots configuration package.

All robot-specific configs should be in this package.

### Arguments
 * name: (required) robot name
 * mini: (default: `false`) launch only nodes required for basic expressions and webui to work
 * rviz: (default: `true`)  Determines iof rviz needs to be launched
 * dev: (default: `false`) Determines if it is runing for development purposes. Currently it launches the generic face tracker from perception node instead of launching the robot specific camera and tracking settings.
### Example: Running Eva

Start robot:
```
roslaunch robots_config robot.launch name:=eva rviz:=True
```

It launches the following nodes:
  * Motor specific nodes (pololu & dynamixel if needed)
  * [pau2motors](https://github.com/hansonrobotics/pau2motors)
  * [basic_head_api](https://github.com/hansonrobotics/basic_head_api)
  * [chatbot](https://github.com/hansonrobotics/chatbot)
  * [Text To Speech](https://github.com/hansonrobotics/tts)
  * [Perception nodes](https://github.com/hansonrobotics/perception) (see geometry.launch)
  * Robot specific camera drivers
  * [pi_vision](https://github.com/hansonrobotics/pi_vision) face tracking
  * [Rosbridge Suite](http://wiki.ros.org/rosbridge_suite)

### Configuration
All configuration files are inside the folder with robot name.
The following configuration files are needed for all nodes to run:

##### bot.properties
Specifies chatbot answers such as name age and etc.

##### config.yaml
Following parmas has to be set:
  * dynamixel controllers settings
  * voice (currently jak, giles and heather supported)
  * lipsync (True/False) - lypsinc enabled or not

##### description.urdf
URDF description of robot. Following frames required:
  * world
  * blender
  * camera

##### expressions.yaml
Expressions for [basic_head_api](https://github.com/hansonrobotics/basic_head_api)
Expressions starting with "vis_" are considered to be visimes.

##### animations.yaml
Animations for [basic_head_api](https://github.com/hansonrobotics/basic_head_api)

##### hardware.launch
Launch file with robot specific motor controllers.

##### motors.yaml
Non-pololu (currently only dynamixel) motors settings pau mappings and etc.

##### pololu
Each pololu board has separate yanl file where all configs including PAU mappings are stored. Pololu node process calibration data and adds to motors parameter in similar format as in motors.yaml

##### tracker.launch
Robot specific camera drivers and pi_vision settings.

### Nodes
#### scripts_launcher.py
Basic script launcher to make simple performances including animations, emotions and tts. See [Sophia Basic Scripts](http://wiki.hansonrobotics.com/w/Sophia_Basic_Scripts) for more information
