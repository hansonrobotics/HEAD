# motors_safety
Safety Layer for robot
### Planned features
 * Do not allow certain motors to go extreme positions
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
