This the a wrapper code for CMT. 

As this code is dependent on the CppMT libary which it wrapps around it's always should be updated before
building this particular code the time being to reflect the latest changes availble in that repo.

        ./hrtool -v

First make sure the CMT library is installed

        git checkout vision_tools
        ./hrtool -b


Launching it from scripts directory to launch it.

        ./vision.sh cmt

To pi_vision pi_vision

        ./vision pi_vision


# Urgent TODO's
* Improve face classifiers as it seems there are no faces in the view most of the time. Triple the size of the image to get smaller images as in the case of exhibition videos. Was done but the
results didn't really translate to better performance as it didn't find faces in most of the case.

* Confiugre tthe codes to the camera as for the quality of the camera it is Full HD 1080P isn't outputing
quality results.


* Configurable dataset where we can trim the sets and remove unnecessary faces and train again without being in
run time.

* 3D location ouput to the relevant topics.

* A node aggregate the relavant topics with time base policy in ROS. 

#TODO
* Write Tests.
* Refactoring the emotime related code in cmt_tracker_node.cpp, Gaze Detection and Face Attention Area.
* Remove unnecessary arguments and parameters in the launcher and code.
* Write a documentation.
* Database based image saving or make the trainined classifier have the option to add new people into it.
* Pose based image saving.
