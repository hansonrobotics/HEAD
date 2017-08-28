# Web UI


WebUI is the Javascript application designed to control robots over the ROS. It uses websockets to connect to ROS.

## Requirements

## Running
##### Production
Running in stable enviroments, will take a while to load for first time as files are beeing minimized:

`webpack --optimize-minimize & node backend/entry.js -p 8000 -c {ROBOTS_CONFIGS_DIR} -r {ROBOT_NAME}`

##### Development
Monitors for file changes and if detected repacks the files and restarts the webserver

`webpack -w -dev & nodemon backend/entry.js -p 8000 -c {ROBOTS_CONFIGS_DIR} -r {ROBOT_NAME}`


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

##### Main Files





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
