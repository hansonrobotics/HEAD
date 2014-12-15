pau2motors
==========

This is a ROS node that receives **PAU** (Physiological Action Unit)
messages and sends out appropriate motor commands. **PAU** or
[**pau2motors/msg/pau.msg**]
(https://github.com/hansonrobotics/pau2motors/blob/master/msg/pau.msg)
messages contain eye, head, neck positions given by 48 coefficients,
the blendshape set.  The blendshape set is compatible with the
Faceshift blendshapes; See [3.4  Tracked Blendshapes]
(http://doc.faceshift.com/faceshift-doc-complete-htmlch3.html#x7-210003.4)
at [doc.faceshift.com](http://doc.faceshift.com). See also [ShapekeyStore.py]
(./src/ShapekeyStore.py)

The purpose of this node is to abstract away robot's hardware from the
nodes that operate in virtual space (possibly [**basic_head_api**]
(https://github.com/hansonrobotics/basic_head_api),
[**robo_blender/modular-dev**]
(https://github.com/hansonrobotics/robo_blender/tree/modular-dev) and [
**blender_api**](https://github.com/hansonrobotics/blender_api)).

##Running

ROS params from
**[robots_config](https://github.com/hansonrobotics/robots_config)**
must be loaded prior to running this node. To achieve this, the node is
either run by one of the launch files in **robots_config** or with a
sequence of commands like this:

```
export ROS_NAMESPACE=/arthur
rosparam load /path_to/robots_config/arthur/config_face.yaml
rosrun pau2motors pau2motors_node.py
```

**pau2motors** node is sensitive to the ROS_NAMESPACE environment
variable and will print an error if it doesn't find suitable ROS
parameters in the current namespace:

```
Couldn't find 'pau2motors' param in namespace '/'.
Change namespace with `export ROS_NAMESPACE=<ns>`
Current valid <ns>: /arthur.
Check robots_config repo to get more config options.
```

##Dependencies

Depends on
**[ros_servo_pololu](https://github.com/hansonrobotics/ros_pololu_servo)**
and **[dynamixel_motor](https://github.com/arebgun/dynamixel_motor)**
packages. These are the package this node sends commands to.

##Notes

Currently the configuration for different robots are stored in the
**[robots_config](https://github.com/hansonrobotics/robots_config)**
package, in yaml config files, in dictionaries called `pau2motors`
(e.g.  [Einstein]
(https://github.com/hansonrobotics/robots_config/blob/master/einstein/config.yaml)).
The `hardware`, `parser` and `function` properties in the config files
refer to classes in this repo's modules **HardwareFactory.py**,
**ParserFactory.py** and **MapperFactory.py**, which may be extended
with new classes.
