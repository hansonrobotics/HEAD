# emotime_ros
this package is ros based emotion detector using emotion detector library called emotime. It subscribes images from cmt 

tracker published topic and process it.

#building emotime-ros
1 Install prerequisites

-clone and build emotime from the following repository 

           https://github.com/Linaf/emotime.git


2. modify LD_LIBRARY_PATH, CPLUS_INCLUDE_PATH to include appropriate emotime library and include files respectively.

     also include opencv library and include files if it is not available in your system yet.

3. clone this repository in to your catkin ws

           cd ~/catkin_ws/src
           
           git clone https://github.com/tesYolan/cmt_tracker.git
          
           git clone https://github.com/Linaf/emotime_ros.git
           
           cd .. && catkin_make
          
           source ./devel/setup.bash 

           catkin_make
          
           source ./devel/setup.bash 

# running cmt_tracker using launch file
       
           roslaunch cmt_tracker cmt_tracker.launch
           
           rqt -s rqt_tracker_view/tracker_plugins
           

           
#running emotime ros using emotime launch file


          roslaunch emotime emotime.launch
          
          
This will publish a topic which appends emotion state on the tracked face
 
         
