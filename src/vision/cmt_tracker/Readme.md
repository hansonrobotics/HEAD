This the a wrapper code for CMT. 

As this code is dependent on the CppMT libary which it wrapps around it's always should be updated before
building this particular code the time being to reflect the latest changes availble in that repo.

        ./hrtool -i # Install dependecies

        ./hrtool -v # Update vision_tools dependecies (CppMT)

        ./hrtool -b # Build the library.


Launching it from scripts directory to launch it.

        ./vision.sh cmt

To pi_vision pi_vision

        ./vision pi_vision

Trimming dataset and creating classifiers before the system is run. Eventhough
during run time we can create classifiers if the aggregated confidence of face during a pritcular period of
time is less that (dynamically recongiruable value) one can add additional faces to train the dataset on and
thus run have a new dataset available during each particular run of openface in face_recongizner node.

To create dataset from the webcam in a stanalone application launch the following node;

        roslaunch cmt_tracker image_to_dataset.launch

Then to train the dataset run

        ./hrtool -d


# Urgent TODO's
* Improving face detection size has really bad impact on the overall process as it takes quite a while for node to get one face
possibily leading to errounous results.


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
