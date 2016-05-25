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
