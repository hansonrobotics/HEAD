from ros_pololu_servo.msg import servo_pololu

class MotorCmd:
  """Represents a partially defined motor command. Builds a ROS msg when given an angle."""

  def msg_angle(self, angle=None):
    """Builds a message given the target angle."""
    msg = servo_pololu()
    msg.id = self.motor_entry['motorid']
    msg.angle = self._saturated(angle or self.target)
    msg.speed = self.motor_entry['speed']
    msg.acceleration = self.motor_entry['acceleration']
    return msg

  def msg_intensity(self, intensity=1):
    """Builds a message given the fractional distance (0 to 1) from rest position to target."""
    angle = (self.target - self.rest) * intensity + self.rest 
    return self.msg_angle(angle)

  def _saturated(self, angle):
    return min(max(angle, self.motor_entry['min']), self.motor_entry['max'])  

  def __init__(self, motor_entry, target=None, rest=None):
    self.motor_entry = motor_entry
    self.target = target or motor_entry['default']
    self.rest = rest or motor_entry['default']