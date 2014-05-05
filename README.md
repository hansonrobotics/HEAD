# Chatbot for ROS

## Installation
```
sudo apt-get install sox
sudo apt-get install ros-groovy-audio-common
```

You will also need to clone [audio-common](https://github.com/ros-drivers/audio_common) into your catkin workspace.

## Running
On a computer:
```
roslaunch chatbot desktop.launch
```

On the robot:
```
rosluanch chatbot robot.launch
```

## Credits
Speech synthesis copied from [gspeech](https://github.com/achuwilson/gspeech) by Achu Wilson.
