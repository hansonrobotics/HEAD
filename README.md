#basic_head_api

###Listens to topics:
+ `/make_face_expr` for msg **MakeFaceExpr.msg** that will trigger the requested PAU face expression to be sent to `/cmd_face_pau`.

+ `/make_coupled_face_expr` for msg **MakeCoupledFaceExpr.msg** that will trigger motor commands to be sent to `/cmd_pololu` for the requested robot and face expression.

+ `/point_face` for msg **PointHead.msg** that will build and send a head rotation msg in quaternion format to `/cmd_neck_pau`.

###Provided services:
+ Call `/valid_face_exprs` with **ValidFaceExprs.srv** to request available face expression strings for use in **MakeFaceExpr.msg**.

+ Call `/valid_coupled_face_exprs` with **ValidCoupledFaceExprs.srv** to do the same for the motor-coupled expressions.

###Notes
Currently the only valid value for **robotname** to put in **MakeCoupledFaceExpr.msg** and **ValidCoupledFaceExprs.srv** is `einstein`.

###Pictures of available PAU expressions
|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/01-happy.png)| ![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/02-sad.png)|
|:-:|:-:|
|Happy|Sad|
|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/03-surprised.png)|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/04-evil.png)|
|Surprised|Evil|
|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/05-afraid.png)|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/06-horrified.png)|
|Afraid|Horrified|
|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/07-annoyed.png)|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/08-innocent.png)|
|Annoyed|Innocent|
|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/09-violent.png)|![](https://raw.githubusercontent.com/hansonrobotics/basic_head_api/master/src/config/10-eureka.png)|
|Violent|Eureka|
