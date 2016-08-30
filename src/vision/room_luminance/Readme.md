# Simple Visual System Needs

This repository contains the code for detecting room luminance in general. Currently, it includes the following features:

* General Room Luminance Detection and Classification.
* Hand/Object Block Recognition

Moreover, the repository will have the following features by the end of development.
* Room Silence
* Room Occupation
* and other simple visual system needs

## Install, Build and Run w/ HEAD
* Refer [HEAD] (https://github.com/hansonrobotics/HEAD#install-build-and-run)

## Run Room Luminance Only
* Navigate to your workspace
* `roslaunch room_luminance room_luminance.launch`

## Topics and Published Messages Type

### Topics
* `/opencog/room_luminance`

### Custom Messages
* Message File: `Luminance.msg`
  * `int8 covered`: checks whether the ROI is covered or not.
  * `int8 sudden_change`: used to publish 1, 0, and 1 for Dark-->Bright, Stable, and Bright-->Dark changes respectively.
  * `string room_light`: string that holds the luminance(Dark, Nominal, and Bright) of a given ROI. 
  * `float32 value`: the amount of light in a frame irresective of the aforementioned states. Range is 1-100.
  * `float32 perc_covered`: This is the percent of screen covered by close enough objects. Very  close objects by themselves may cover the camera and affect it to have very small luminance. On the contrary, the robot camera could be covered without affecting the amount of light applied on the camera. In this scenario, the robot must know that it is covered by some object/hand.

