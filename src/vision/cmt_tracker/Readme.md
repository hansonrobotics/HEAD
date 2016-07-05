This the a wrapper code for CMT. 

As this code is dependent on the CppMT libary which it wrapps around it's always should be updated before
building this particular code the time being to reflect the latest changes availble in that repo.

        ./hrtool -i # Install dependecies

        ./hrtool -v # Update vision_tools dependecies (CppMT)

        ./hrtool -b # Build the library.


Launching it from scripts directory to launch it.

        ./vision.sh cmt

To pi_vision pi_vision

        ./vision.sh pi_vision

Trimming dataset and creating classifiers before the system is run. Eventhough
during run time we can create classifiers if the aggregated confidence of face during a pritcular period of
time is less that (dynamically recongiruable value) one can add additional faces to train the dataset on and
thus run have a new dataset available during each particular run of openface in face_recongizner node.

Then to train the dataset run

        ./vision.sh cmt_offline


# Urgent TODO's
* 3D location ouput to the relevant topics.

* Merging overlapping tracking instances created when a face moves.

* Inititial threshold configuration to make sure a random false positive doesn't wait large number of frame before deletion.

* Enable gaze, attention and emotime output into the the relevant code.

* Increase the training of the dlib object detetor or utilize higher FP opencv classifiers to reinforce temporay trackers
to push them to the main tracker queue.

#TODO
* Write Tests.
* Remove unnecessary arguments and parameters in the launcher and code so to make it entirely configurable.
* Write a documentation and clear API to trigger adding faces to the dataset.
* Resolve how to deal with the openface current limitiation that training can be done from the local image.
* Pose based image saving.
