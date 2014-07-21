pau2motors
==========

A node that receives **PAU** (Physiological Action Unit) messages and sends out appropriate motor commands. **PAU** or **pau2motors.msg.pau** messages include a single face expression and eye, head, neck positions in Faceshift format - a robot-agnostic way.

##Dependencies

Depends on **[ros_servo_pololu](https://github.com/hansonrobotics/ros_pololu_servo)** and **[dynamixel_motor](https://github.com/arebgun/dynamixel_motor)** packages. These are the package this node sends commands to.

##Notes

Modules **HardwareFactory.py**, **ParserFactory.py** and **MapperFactory.py** may be extended with new classes, which may then be referred from the config file's **hardware**, **parser** and **function** properties. New type of motor commands, non-linear mapping functions and one-to-many Motor-PAU bindings can be implemented this way. 
