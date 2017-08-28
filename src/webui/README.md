# Web UI
=======

WebUI is the Javascript application designed to control robots over the ROS. It uses websockets to connect to ROS.

## Requirements

## Running

## Deveoper information
### Backend
NodeJS backend application: [source](backend/)
### Fronend 
Javascript Client application: [source](client)
##### Framework and Main Libraries
The core libraries:
* Backbone
* MarionetteJS
* Bootstrap
* Jquery
* RequireJS
* Webpack





## Troubleshooting

Sometimes not all topics subscribed or beiiing publsihed after refresh. See the issue https://github.com/RobotWebTools/rosbridge_suite/issues/138

Need patch for rosbridge module. By default the path on Ubuntu:
`/opt/ros/indigo/lib/python2.7/dist-packages/rosbridge_library/internal/publishers.py`

```
A quick hack to prevent the problem is commenting out topic unregistration in pulibhser.py

311# if not self._publishers[topic].has_clients():
312#           self._publishers[topic].unregister()
313#            del self._publishers[topic]
```
