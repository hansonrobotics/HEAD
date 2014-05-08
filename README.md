# Chatbot for ROS

## Installation
```
sudo apt-get install sox
sudo apt-get install ros-groovy-audio-common
sudo apt-get install python pip
pip install flask
```

You will also need to clone [audio-common](https://github.com/ros-drivers/audio_common) into your catkin workspace, as well as [rosbridge-suite](https://github.com/RobotWebTools/rosbridge_suite). rosbridge-suite can also be installed with ros-groovy-rosbridge-suite, but the Debian repository is broken, which is why you need to clone the repository to your workspace instead.

## Running
### On a computer
Set the ROS Master URI to point to the robot.
Open scripts/templates/index.html, and change the WebSocket url to your computer. Set the port to 9090.
```
roslaunch chatbot web.launch
```

### On the robot
```
roslaunch chatbot robot.launch
```

## Credits
Speech recognition uses the [Web Speech API](https://www.google.com/intl/en/chrome/demos/speech.html) in Chrome.

[Flask](http://flask.pocoo.org/) is used to run a simple web server.

[Robot Web Tools](http://robotwebtools.org/) is used to communicate with the ROS Master from a website.

The chatbot is based on the free [Alice AIML](https://code.google.com/p/aiml-en-us-foundation-alice/) set.

Google Translate is unofficially used to synthesize voices from text. [SoX](http://sox.sourceforge.net/) is used to convert the sound file.
