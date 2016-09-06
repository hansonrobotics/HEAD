# HEAD

[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-HEAD)](http://61.92.69.39:8080/view/hansonrobotics/job/ci-HEAD/)

**H**anson **E**nvironment for **A**pplication **D**evelopment

This repository contains the integrated code for controlling and
interacting with various Hanson Robotics robot heads, including a
pure-software implementation. It includes the full theatrical 
performance pipeline and infrastructure:

* Vision and audio perception, via ROS webcam nodes.
* Face detection, for seeing faces in the room.
* Blender robot model, for animating movements and facial expressions.
* Behavior tree, for scripting performaces.
* Motor control ROS nodes, for controlling the physical robot.

## Prerequisites

### Hardware

* Powerful graphics card

### Software

 * Ubuntu 14.04 is installed. **Only Ubuntu 14.04 is supported**.
 * X Server is running.
 * Use root or create user with the ability to gain root privileges.

## TL;DR

Install dependencies, get source code, build

`sudo apt-get update && sudo apt-get install wget && wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool -O /tmp/hrtool && chmod +x /tmp/hrtool && /tmp/hrtool -iGBy`

Then run

`~/hansonrobotics/HEAD/scripts/dev.sh`

## Install, build and run

* Install wget

sudo apt-get install wget (Run apt-get update first if wget is not found)

* Get hrtool

`wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool -O /tmp/hrtool`

* Make it executable

`chmod +x /tmp/hrtool`

* Optionally, set the workspace. The default workspace is `~/hansonrobotics`

`/tmp/hrtool -w <workspace>`

* Install dependencies

`/tmp/hrtool -i`

* Get HEAD and OpenCog source code

`/tmp/hrtool -G`

* Build HR workspace and OpenCog

`/tmp/hrtool -B`

* Remove hrtool. After the source code is built, this script is not
  needed anymore. A copy of the script is in `<workspace>/scripts/hrtool/hrtool`

`rm /tmp/hrtool`

* Run

`cd ~/hansonrobotics/HEAD/scripts && ./dev.sh`

* Open a web browser

HTTP: http://127.0.0.1:8000/ or HTTPS: https://127.0.0.1:4000/

## Other options of hrtool

**The hrtool script is located in `~/hansonrobotics/HEAD/scripts`**

* `-s` Build a single ROS package and it's related dependecies.

* `-v` Get the latest code for the cmt vision system.

* `-d` Get the latest OpenCog source code (developer mode -- this is the unstable branch).
   **Use this option only if you want to get the latest OpenCog stack. It will change the
     remote of git repositories for OpenCog to the [OpenCog](https://github.com/opencog) domain.**

* `-U` Update the code to the latest, including both HEAD and the OpenCog stack.

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
Install pip3 (again): `apt-get install -y --reinstall python3-pip`

### Workspace warning
> [WARN] The workspace configured /home/ubuntu/hansonrobotics doesn't match the your working path /
> Continue using /home/ubuntu/hansonrobotics? [y/N]

If you are using `/tmp/hrtool`, you can ignore this error. From this point forward, 
use `<workspace>/scripts/hrtool`

