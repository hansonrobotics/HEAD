FROM opencog/ros-indigo-blender

RUN apt-get install -y ros-indigo-rosbridge-server

WORKDIR /catkin_ws/src
RUN git clone https://github.com/hansonrobotics/ros_motors_webui.git

#rosbridge websockets
EXPOSE 9090
#Webserver
EXPOSE 8000