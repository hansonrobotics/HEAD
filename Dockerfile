#docker.io run -v /dev/ttyACM0:/dev/ttyACM0 -p 8000:8000 -p 9090:9090 -it myROS

FROM opencog/ros-indigo-blender

RUN apt-get install -y ros-indigo-rosbridge-server

WORKDIR /catkin_ws/src
RUN git clone https://github.com/hansonrobotics/ros_motors_webui.git

#rosbridge websockets
EXPOSE 9090
#Webserver
EXPOSE 8000

ENTRYPOINT /bin/bash -l -c "source /.bashrc && roscore & sleep 5 && (source /.bashrc && roslaunch rosbridge_server rosbridge_websocket.launch & source /.bashrc && rosrun ros_pololu_servo ros_pololu_servo_node) & cd /catkin_ws/src/ros_motors_webui && python -m SimpleHTTPServer"