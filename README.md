
# Eva Blender Animation API #

This repository contains an animated model of the Eva head, as
a Blender file.  The head can be animated by selecting from a menu of
factial expressions and head movements. These can be activated from the
blender controls, or by sending specially crafted messages to a listener
daemon.

# Demo #

This video shows what is currently possible: https://www.youtube.com/watch?v=s8Kwcor3CvY

# Running #

Pre-requisites: you must have a working version of Blender.

 * Start Blender, load Eva.blend.
 * At bottom center, click on "Run Script".
 * This will bring up a RigControl panel on the right.
 * Click on "Start Animation".

# Command Listener #

Comands may be sent by ... XXX?  Must start command listener first ... 


# Design #

The loader.py file defines and loads the rigControl for blender.

The animationManager.py file does ?? create and destroy of new gestures?
what does that mean? 

The blenderPlayback.py file runs the actual animations. 

The blenderUI.py file defines the tracking and emotions commands ... 

Where are the gestures hooked up to things? i.e. blink, nod-1, nod-2,
etc.  How does the command listener get these, and send them on?


# Copyright #

Copyright (c) 2014 Hanson Robotics 


(I assume, or is this (c) Mike Pan also/instead-of ??)
