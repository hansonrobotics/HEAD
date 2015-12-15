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
 2. Run `./app/__init__.py`, then open [localhost:5000](http://localhost:5000).

Troubleshooting
----------
Sometimes not all topics subscribed or beiiing publsihed after refresh. 

https://github.com/RobotWebTools/rosbridge_suite/issues/138

Need patch for rosbridge module.
By default the path on ubuntu:
`/opt/ros/indigo/lib/python2.7/dist-packages/rosbridge_library/internal/topics.py`


