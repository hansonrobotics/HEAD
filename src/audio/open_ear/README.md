itf_open_ear
==========

This package launches a talker which instantiates an audio processor from the modified ros_open_ear package and publishes the detected emotions on /affect_pub and /emo_pub.

Prerequisites
-------------
See the geni-lab/ros_open_ear package for instructions on how to install that.

If you are having trouble getting your audio input device to work, try getting the latest version of PortAudio at http://www.portaudio.com/download.html (pa_stable_v19_20140130.tgz worked for us). Unpack it, make, sudo make install.

You'll also have to copy the models and config folders from where you installed ros_open_ear to the root of your catkin workspace. Assuming ~/ros_open_ear for that and ~/catkin_ws for your catkin workspace root:

cd ~/catkin_ws
cp -r ~/ros_open_ear/config .
cp -r ~/ros_open_ear/models .

And you'll have to run catkin_make to compile itf_open_ear of course.

Usage
-----
To run (from the root of your catkin workspace)

rosrun beginner_tutorials talker -C config/emobase_live4.conf

OR

rosrun begginer_tutorials talker -C /home/yourusername/catkin_ws/config/emobase_live4.conf

To check if it's working:

rostopic echo /emo_pub

Notes
-----
None.
