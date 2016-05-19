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

