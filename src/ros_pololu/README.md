# ros_pololu
Driver for Pololu motors with ROS.
Controls Pololu Maestro as well as Micro Serial Servo Controller 

### Requirements 
[Pololu Motors](https://github.com/hansonrobotics/pololu-motors) needs to be installed

### Config 
Config is loaded from static file passed as private parameter *~pololu_motors_yaml*
Example: https://github.com/hansonrobotics/robots_config/blob/master/eva/face.yaml
Single entry should look like this:
```
Joint1:
  motor_id: 0
  init: 1500
  min: 1000
  max: 1900
  speed: 0
  acceleration: 0
  callibration: 
      min_angle: -45
      min_pulse: 1000
      max_angle: 45
      max_pulse: 1900
```
Only *motor_id, min, max, init*  are required. If callibration is not provided the motor is configured in the way that its range is 90 degrees and init value has 0 degrees.
Once calibrated motors are appended to the ROS param server *motors* parameter, with calibrated values added. 
Additional settings can be added to configs, and those will be later passed to motors param for other nodes to process.

In addition if motors are not configured, the motor_id can be sent as joint name, and the angle will be mapped to 820 - 2280 pulse range which corresponds to -90 - 90 degrees angles.
