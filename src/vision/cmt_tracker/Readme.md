This the a wrapper code for CMT. 

First make sure the CMT library is installed

        git checkout vision_tools
        ./hrtool -b


Launching it from sophia directory to launch it. 

    roslaunch tracker-single-cam.launch    #This cause it's not been integrated to the full stack yet. 
    

#TODO
* Make sure faces isn't tracked while it's in a process it being tracked in another application.
* Write Tests.
* Refactoring the emotime related code in cmt_tracker_node.cpp
* Include training in __init__ in face_recognizer.py to remove deleted classifiers.
* Include sophia and pkd photos to start the classifier test.
* Remove unnecessary arguments and parameters in the launcher and code.
* Write a documentation.
* Setting a Working location for places to save images.
* Database based image saving or make the trainined classifier have the option to add new people into it.
* Pose based image saving.

# Urgent TODO's
* Threading the cmt instances in process_mode in the main CppMT handlers. As this affects the speed in which the people are percieved.
* Triple the size of the image to get smaller images as in the case of exhibition videos.
* Improve face classifiers as it seems there are no faces in the screen.
* Confidence based reinfocemnt for simple overlap. That is for every non
    overlap there needs to be a confidence decrease otherwise
    there would be garabage results for a
    while in the system.
* Incoprate a threshold of two to three frames for the tracker to readjust as the turnover
rate is pretty much high.

