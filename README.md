# motors_safety
Safety Layer for robot
### Current Features
##### Prevent from conflicting extreme position:
Such rules prevent motor to go certain positions if specified motors are in their extreme positions.
###### Config
Reads the `safety_rules` and `motors` params from param server in current namespace.
`motors` param is global between the HR ROS nodes and it has configs for all 
`safety_rules` param should be specified in [robots_config](https://github.com/hansonrobotics/robots_config) package `config.yaml` for the specific robot
Example:
```
safety_rules:
    Smile-L:
        - type: prevent
          direction: max
          extreme: 0.5
          depends: Frown-L
          dep_dir: max
          dep_extreme: 0.5
```
It means if `Smile-L` servo has command to move to more than 50% of its DOF from neutral towards maximum direction, it will check if Frown_L is not exceeding the 50% of DOF to its `max` direction. If it does, than `Smile_L` will be limited to 50% and wont go further.





### Planned features

 * Allow extreme positions only for certain amount of time
 * Read motor sensors and adjust its position/torque

### Motor Interdependencies
 * Write out dependency on head diagram
 * Define servo safety interdependencies
 * Write motor compressor profile

### Motor Testing
 * Unit test each motor seperately
 * System test motor interdependencies for mouth and jaw motors
 * Measure sensor readings for tempurature, torque, position
 * Safety runs in background during robot performances to make sure safety is kept for motors

### Physics Testing (long term plan)
 * Test the skin physics
