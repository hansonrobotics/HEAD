# HEAD

Hanson Environment for Application Development

This repository contains the integrated code for controlling and
interacting with many Hanson Robotics robot heads. It includes the
full performance pipeline and infrastructure:

* Perception, via ROS webcam nodes.
* Face detection, for seeing faces in the room.
* Blender robot model, for gracefully controlling facial expressions.
* Behavior tree, for scripting performaces.
* Motor control ROS nodes, for controlling the physical robot.

## Prerequisites

 * **Only Ubuntu 14.04 is supported**

## Install, build and run

* Get hrtool

`wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool`

* Make it executable

`chmod +x hrtool`

* Set the workspace, default workspace is ~/hansonrobotics (optional)

`./hrtool -w <workspace>`

* Get HEAD and OpenCog source code

`./hrtool -G`

* Install Dependencies

`./hrtool -i`

* Build HR workspace and OpenCog

`./hrtool -B`

* Build a single ROS package and it's related dependecies

`./hrtool -s package_name`


* Run

`cd ~/hansonrobotics && ./dev.sh`

* Run vision tools

`./vision.sh cmt` #For running with cmt/pi_vision

* Get the latest vision codes relating to cmt.

`./hrtool -v`

* Open web browser

HTTP: http://127.0.0.1:8000/ or HTTPS: https://127.0.0.1:4000/

* Remove hrtool. After the source code is built we don't need this script anymore. 

`rm ./hrtool`

