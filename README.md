ros_motors_webui
=======

ros_motors_web_ui is the html/javascript application designed to control robots over the ROS. It uses websockets to connect to ROS.

Requirements
------------

 1. Rosbridge server. See http://wiki.ros.org/rosbridge_suite
 2. Browser supporting websockets: http://caniuse.com/#feat=websockets

Running instructions
-------------

Application can run on any webserver, which could connect to the rosbridge websocket. Currently it is assumed that rosbridge server are on same server.

 1. Start rosbridge server: `roslaunch rosbridge_server rosbridge_websocket.launch`
 2. Run `./flaskserver.py`, then open [localhost:5000](http://localhost:5000).

Components and ROS topics
----------
#### index.html (Emotions)
 1. Gets the list of expressions from ROS *valid_coupled_face_exprs*  topic served by [basic_head_api](https://github.com/hansonrobotics/basic_head_api) node
 2. Sends the expression and intensity to the */make_coupled_face_expr* to same [basic_head_api](https://github.com/hansonrobotics/basic_head_api) node
 3. The joystick control sends the head direction to */point_head* module.
 
 ----------

#### motors.html (Manual Control)
Controls individual motors manually. loads config from : `/configs/` dir.
Currently supported:

 1. Dynamixels
 2. Pololulu using [ros_pololu_servo node](https://github.com/hansonrobotics/ros_pololu_servo)

----------
