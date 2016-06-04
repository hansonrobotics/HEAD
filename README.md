# HEAD

[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-HEAD)](http://61.92.69.39:8080/view/hansonrobotics/job/ci-HEAD/)

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

* Run

`cd ~/hansonrobotics && ./dev.sh`

* Open web browser

HTTP: http://127.0.0.1:8000/ or HTTPS: https://127.0.0.1:4000/

* Remove hrtool. After the source code is built we don't need this script anymore. 

`rm ./hrtool`

## Checkout latest OpenCog (developer mode)

`./hrtool -b`

This option is used when you want to get the latest OpenCog stack. It will set the remote of git repositories of OpenCog to [OpenCog](https://github.com/opencog) domain.

