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

* Install wget

sudo apt-get install wget (Run apt-get update first if wget is not found)

* Get hrtool

`wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool -O /tmp/hrtool`

* Make it executable

`chmod +x /tmp/hrtool`

* Set the workspace, default workspace is ~/hansonrobotics (optional)

`/tmp/hrtool -w <workspace>`

* Install Dependencies

`/tmp/hrtool -i`

* Get HEAD and OpenCog source code

`/tmp/hrtool -G`

* Build HR workspace and OpenCog

`/tmp/hrtool -B`

* Remove hrtool. After the source code is built we don't need this script anymore.

`rm /tmp/hrtool`

* Run

`cd ~/hansonrobotics/HEAD/scripts && ./dev.sh`

* Open web browser

HTTP: http://127.0.0.1:8000/ or HTTPS: https://127.0.0.1:4000/

## Some other options of hrtool

**The same hrtool script is included in ~/hansonrobotics/HEAD/scripts**

* `-s` Build a single ROS package and it's related dependecies.

* `-v` Get the latest vision codes relating to cmt.

* `-d` Checkout latest OpenCog (developer mode).

> **This option is used when you want to get the latest OpenCog stack. It will set the remote of git repositories of OpenCog to [OpenCog](https://github.com/opencog) domain.**

* `-U` Update the code to the latest including HEAD and OpenCog stack.

* `-c` Clean up OpenCog installation and cache.

* `-h` Show more options.

## Other options for running HEAD.

`cd ~/hansonrobotics/HEAD/scripts`

* Run with cmt as default camera tracker

`./dev.sh --cmt`

* Run with OpenCog chatbot instead of AIML based chatbot.

`./dev.sh --oc` 

* Run vision tools

`./vision.sh cmt` #For running with cmt/pi_vision

## Troubleshooting

### pip3 is not found
Reinstall pip3 `apt-get install -y --reinstall python3-pip`

