# HEAD

[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-HEAD)](http://61.92.69.39:8080/view/hansonrobotics/job/ci-HEAD/)

**H**anson **E**nvironment for **A**pplication **D**evelopment

This repository contains the integrated code for controlling and
interacting with various Hanson Robotics robot heads, including a
pure-software implementation. It includes the full theatrical 
performance pipeline and infrastructure:

* Vision and audio perception, via ROS webcam and microphone nodes.
* Face detection, for seeing faces in the room.
* Blender robot model, for animating movements and facial expressions.
* Behavior tree, for scripting performaces.
* Motor control ROS nodes, for controlling the physical robot.

## Prerequisites

### Hardware

* Intel/AMD workstation or laptop w/4GB or more RAM, 20GB disk, and 
  a fairly powerful graphics card.
* If a dedicated workstation is not available, consider using LXC, so
  as to not mess up your base OS.

### Software

 * Ubuntu 14.04 is installed. **Only Ubuntu 14.04 is supported**.
 * X Server is running.
 * Use root or create user with the ability to gain root privileges.

## TL;DR

Install dependencies, get source code, build. This may take several hours.

`sudo apt-get update && sudo apt-get install wget && wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool -O /tmp/hrtool && chmod +x /tmp/hrtool && /tmp/hrtool -iGBy`

Then run

`~/hansonrobotics/HEAD/scripts/dev.sh`

## Native install, build and run
Use these instructions, if you are willing to install the system "natively" 
on your machine (i.e. into the root file system). Otherwise, use the LXC instructions,
further down below.  Install may take several hours; a fast internet connection is
strongly recommended.

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


* Remove hrtool. After the source code is built, this script is not
  needed anymore. A copy of the script is in `<workspace>/scripts/hrtool/hrtool`

`rm /tmp/hrtool`

* Build the HR workspace and OpenCog

`~/hansonrobotics/HEAD/scripts/hrtool -B`

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

## LXC Install, build and run directions.
LXC -- Linux Containers -- is a tool that allows you to run a virtualized
operating system container on your machine.  Because each container can
have a different root file system, it is convenient for experiments, and
for isolating your main system from robot-related configuration issues.
One can run multiple containers, too -- even dozens or more.

Start by installing LXC: review [LXC Getting Started](https://linuxcontainers.org/lxc/getting-started/)
```
sudo apt-get install lxc
lxc-create -t download -n my-hr-HEAD -- -d ubuntu -r trusty -a amd64
lxc-start -n my-hr-HEAD -d
lxc-attach -n my-hr-HEAD --clear-env
(then, at the prompt of the attached container):
apt-get install openssh-server
cd /home/ubuntu
mkdir .ssh
cd .ssh
vi authorized_keys
(add your public key)
chmod go-rwx . authorized_keys
chown ubuntu:ubuntu . authorized_keys
passwd ubuntu
(change passwd -- this is needed because hrtool needs it)
```
After the above, you should be able to ssh into your new container.
After doing so, the instructions are essentially the same as the
native install.  These are repeated below.
```
lxc-ls -f
ssh -X ubuntu@10.0.3.239  (or whatever IP address lxc-ls shows)
sudo apt-get install wget
wget https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hrtool -O /tmp/hrtool
chmod +x /tmp/hrtool
/tmp/hrtool -i
/tmp/hrtool -G
rm /tmp/hrtool
~/hansonrobotics/HEAD/scripts/hrtool -B
```
To get video (the webcam, used for vision) working from an LXC container,
you will need to do some config file tinkering.  Edit the file
`~/.local/share/lxc/my-hr-HEAD/config` and add the line:
```
lxc.cgroup.devices.allow = c 81:0 rwm
```


## Troubleshooting

### pip3 is not found
Install pip3 (again): `apt-get install -y --reinstall python3-pip`

### Workspace warning
> [WARN] The workspace configured /home/ubuntu/hansonrobotics doesn't match the your working path /
> Continue using /home/ubuntu/hansonrobotics? [y/N]

If you are using `/tmp/hrtool`, you can ignore this error. From this point forward, 
use `<workspace>/scripts/hrtool` instead.

