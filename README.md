##ros_motors_webui

Web interface to control Mini-Einstein's face and neck using **ros_servo_pololu**, **basic_head_api** and **rosbridge_server** packages.

###Installation

Download and run a full docker image:

1. Plug in the Pololu controller.
2. Run `docker.io run --privileged -v /dev/ttyACM0:/dev/ttyACM0 -p 8000:8000 -p 9090:9090 -it gaboose/ros-indigo-hanson-webui`.
3. Wait for the image to download and run.
4. Open **http://127.0.0.1:8000**.

Or for latest changes build the Dockerfile inside or install the following packages manually:

+ **ros_servo_pololu**: https://github.com/hansonrobotics/ros_pololu_servo
+ **basic_head_api**: https://github.com/hansonrobotics/basic_head_api
+ **Rosbridge**: `apt-get install ros-indigo-rosbridge-server`

###Usage

Go to the directory containing **index.html**, run **run.sh**, then open **http://127.0.0.1:8000**

###Notes

Avoid running the docker image with no Pololu controller plugged in. A folder named **/dev/ttyACM0** will be created, which interferes with access to the real **/dev/ttyACM0**.

**rosbridge_server** is assumed to be located at the same IP address as the webpage, using **document.domain** js property.
