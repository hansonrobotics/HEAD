# Robot configuration files

Robots configuration package.

All robot-specific configs should be in this package.

### Example: Running Arthur

Start robot:
```
roslaunch robots_config robot.launch name:=arthur rviz:=True
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
Specifies chatbot answers suchas name age and etc.

##### config.yaml
Following parmas has to be set:
  * dynamixel controllers settings
  * voice (currently jak, giles and heather supported)
  * lipsync (True/False) - lypsinc enabled or not
  * pau2motors settings to map motors with pau coefficients

##### description.urdf
URDF description of robot. Following frames required:
  * world
  * blender
  * camera

##### expressions.yaml
Expressions for [basic_head_api](https://github.com/hansonrobotics/basic_head_api)
Expressions starting with "vis_" are considered to be visimes.


##### hardware.launch
Launch file with robot specific motor controllers.
_Each of pololu controllers should have sparate yaml files for callibration data_

##### motors.yaml
Individual motors settings for WebUI and basic_head_api

##### tracker.launch
Robot specific camera drivers and pi_vision settings.









