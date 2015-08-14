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
nodes that operate in virtual space (possibly [**blender_api_msgs**]
(https://github.com/hansonrobotics/blender_api_msgs)
and [**blender_api**](https://github.com/hansonrobotics/blender_api)).

##Running

ROS params from
**[robots_config](https://github.com/hansonrobotics/robots_config)**
must be loaded prior to running this node. To achieve this, the node is
either run by one of the launch files in **robots_config** or with a
sequence of commands such as this:

```
export ROS_NAMESPACE=/han
rosparam load /path_to/robots_config/han/config.yaml
rosrun pau2motors pau2motors_node.py
rosrun topic_tools mux /han/neck_pau cmd_neck_pau /blender_api/get_pau mux:=neck_pau_mux
rosservice call neck_pau_mux/select /blender_api/get_pau
```
The mux that is started and switched on in the last two lines serves
to attach the output of the Sophia blender model to the topics that the
pau2motors listens to. The head_pau should also be attached.


**pau2motors** node is sensitive to the ROS_NAMESPACE environment
variable and will print an error if it doesn't find suitable ROS
parameters in the current namespace:

```
Couldn't find 'pau2motors' param in namespace '/'.
```
The **robots_config** repo contains the various valid namespaces;
these include `han`, `sophia`, `george` and `arthur`. Set the namespace
by saying `export ROS_NAMESPACE=<ns>` before the rosrun.

##Dependencies

Depends on
**[ros_pololu](https://github.com/hansonrobotics/ros_pololu)**
and **[dynamixel_motor](https://github.com/arebgun/dynamixel_motor)**
packages. These are the packages that this node sends commands to.

##Notes

Currently, the configuration for different robots are stored in the
**[robots_config](https://github.com/hansonrobotics/robots_config)**
package, in yaml config files, in dictionaries called `pau2motors`
(e.g.  [Han]
(https://github.com/hansonrobotics/robots_config/blob/master/han/config.yaml)).
Node gets the **motors** param from param server.
**HardwareFactory.py** makes the motor controller from motor entries.
The `parser` and `function` properties in the 
refer to classes in this repo's modules ,
**ParserFactory.py** and **MapperFactory.py** which may be extended
with new classes.
