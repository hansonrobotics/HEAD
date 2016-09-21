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
Sometimes not all topics subscribed or beiiing publsihed after refresh. See the issue https://github.com/RobotWebTools/rosbridge_suite/issues/138

Need patch for rosbridge module. By default the path on Ubuntu:
`/opt/ros/indigo/lib/python2.7/dist-packages/rosbridge_library/internal/publishers.py`

```
A quick hack to prevent the problem is commenting out topic unregistration in pulibhser.py

311# if not self._publishers[topic].has_clients():
312#           self._publishers[topic].unregister()
313#            del self._publishers[topic]
```
