pau2motors
==========

A node that receives **PAU** (Physiological Action Unit) messages and sends out appropriate motor commands.

##Dependencies

Depends on **[ros_faceshift](https://github.com/hansonrobotics/ros_faceshift)** package, because the node currently uses **ros_faceshift/fsMsgTrackingState.msg** as the **PAU** message, which describes face expression, and eye, head, neck position in Faceshift format - a robot-agnostic way.

Depends on **[ros_servo_pololu](https://github.com/hansonrobotics/ros_pololu_servo)** package. That is the package this node sends commands to.

##Notes

Modules **ParserFactory.py** and **MapperFactory.py** may be extended with new classes, which may then be referred from the config file's **parser** and **function** properties. Non-linear mapping functions and one-to-many Motor-PAU bindings can be made this way. 
