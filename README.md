##ros_motors_webui

Web interface to control a separate motors of a robot face using **ros_servo_pololu** and **rosbridge_server** packages.

###Usage

Go to the main directory and run `python -m SimpleHTTPServer`, then open **[http://127.0.0.1:8000](http://127.0.0.1:8000)**

###Notes

**rosbridge_server** is hardcoded to be located at **127.0.0.1**. This can be changed inside the **index.html**.
