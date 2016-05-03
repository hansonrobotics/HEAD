#basic_head_api
Allows to use static expressions with the intensity level. Used to define and control visimes

###Dependencies
+ Requires hansonrobotics/ros_pololu_servo
+ dynamixel-msgs
+ hansonrobotics/pau2motors

### Configuration 
Uses following params from param server (within same namespace) for configuration:
+ `expressions` : see [expressions.yaml](https://github.com/hansonrobotics/robots_config/blob/master/han/expressions.yaml)
+ `motors` : see [motors.yaml](https://github.com/hansonrobotics/robots_config/blob/master/han/motors.yaml)

###Running
   `rosrun basic_head_api head_ctrl.py`

###Listens to topics:
+ `make_face_expr` for msg **MakeFaceExpr.msg** (e.g. ["happy", 0.7])
   that will trigger the requested PAU face expression to be sent to
   `cmd_face_pau`.

+ `point_head` for msg **PointHead.msg** (e.g. [0.785, 0.0, 0.0] to
   turn the head 45 deg (0.785 rad) to the side) that will build and
   send a head rotation msg in quaternion format to `cmd_neck_pau`. **DEPRECIATED**

###Provided services:
+ Call `/valid_face_exprs` with **ValidFaceExprs.srv** to request
  available face expression strings for use in **MakeFaceExpr.msg**.



