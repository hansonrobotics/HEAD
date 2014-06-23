# Run with: docker run --privileged -v /dev/ttyACM0:/dev/ttyACM0 -p 8000:8000 -p 9090:9090 -it gaboose/ros-indigo-hanson-webui

FROM opencog/ros-indigo-dev
MAINTAINER Gabrielius Mickevicius "gabrielius.m@gmail.com"

RUN apt-get -y update
RUN apt-get -y install python3-yaml python3-pip
RUN pip3 install rospkg catkin_pkg

RUN echo source /catkin_ws/devel/setup.bash >> /.bashrc && \
    echo -e "\e[1;34m[$SELF_NAME] catkin devel setup\.bash sourced\e[0m"

RUN echo source /.bashrc >> /.bash_profile

WORKDIR /catkin_ws/src
RUN cp /opt/ros/indigo/setup.sh /etc/profile.d/ros_indigo.sh
ENV PYTHONPATH /opt/ros/indigo/lib/python2.7/dist-packages
RUN /usr/bin/python3 /opt/ros/indigo/bin/catkin_init_workspace

RUN git clone https://github.com/hansonrobotics/ros_pololu_servo

# ROS general instalation ends here
# Install expression controller and webui

RUN apt-get install -y ros-indigo-rosbridge-server

WORKDIR /catkin_ws/src
RUN git clone https://github.com/hansonrobotics/pau2motors.git
RUN git clone https://github.com/hansonrobotics/basic_head_api.git
RUN git clone https://github.com/hansonrobotics/ros_motors_webui.git

WORKDIR /catkin_ws
RUN bash -l -c "/usr/bin/python3 /opt/ros/indigo/bin/catkin_make"

#rosbridge websockets
EXPOSE 9090
#Webserver
EXPOSE 8000

ENTRYPOINT bash -l -c "cd /catkin_ws/src/ros_motors_webui; ./run.sh; bash"