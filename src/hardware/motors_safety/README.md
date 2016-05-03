# motors_safety
Safety Layer for robot
### Current Rules

###### Config
Reads the `safety_rules` and `motors` params from param server in current namespace.
`motors` param is global between the HR ROS nodes and it has configs for all 
`safety_rules` param should be specified in [robots_config](https://github.com/hansonrobotics/robots_config) package `config.yaml` for the specific robot.

##### Prevent from conflicting extreme position:
Such rules prevent motor to go certain positions if specified motors are in their extreme positions.
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

##### Extreme position for limited time
These kind of rules prevent motor from staying in its extreme position for extended period of time. Example config:
```
safety_rules:
    Smile-L:
        - type: timing
          direction: max
          extreme: 0.6
          t1: 1
          t2: 1
          t3: 3
          t4: 1
```
For motor which stays over extreme position more than time `t1`the limit will be gradually set to its threshold (`extreme`) in `t2` time. Limit will stay for time `t3` and then gradually will be removed in time `t4`. `Extreme` is threshold for motor to be considered in extreme position. It is relative position between Neutral and motor extreme in specified `direction`

##### Based on load
For Dynamixel motors the motor position can be adjusted if extreme motor load is applied for period of time. Example config:
```
safety_rules:
    Smile-L:
        - type: load
          direction: min
          extreme: 0.2
          motor_id: 8
          rest: 0.1
          t1: 1
          t2: 4
```
`motor_id` specifies the dynamixel motor_id, `extreme` motor load. `rest` the desired load in the rest mode. If motor stays under `extreme` load for more than `t1` time , then this rule will move motor towards neutral position until  the rest laod is reached and will stay in `rest` load for `t2` time.
 
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
