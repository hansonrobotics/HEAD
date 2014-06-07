##ros_motors_webui

Web interface to control a separate motors of a robot face using **ros_servo_pololu** and **rosbridge_server** packages.

###Dependencies

Install rosbridge server with `apt-get install ros-indigo-rosbridge-server`.

Run rosbridge server `roslaunch rosbridge_server rosbridge_websocket.launch`.

###Usage

After installing dependencies go to the main directory and run `python -m SimpleHTTPServer`, then open **[http://127.0.0.1:8000](http://127.0.0.1:8000)**

Or do a full docker installation:

1. Plug in the pololu controller.
2. Run `docker.io run --privileged -v /dev/ttyACM0:/dev/ttyACM0 -p 8000:8000 -p 9090:9090 -it gaboose/ros-indigo-hanson-webui`.
3. Wait for the image to download and run.
4. Open **[http://127.0.0.1:8000](http://127.0.0.1:8000)**.

###Notes

**rosbridge_server** is assumed to be located at the same IP address as the webpage using **document.domain** js property.
