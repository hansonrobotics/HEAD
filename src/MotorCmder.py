from ros_pololu_servo.msg import MotorCommand

class MotorCmder:
  """Represents a partially defined motor command. Builds a ROS msg when given an angle."""

  def msg_angle(self, angle=None):
    """Builds a message given the target angle."""
    msg = MotorCommand()
    msg.joint_name = self.motor_entry['name']+"@"+self.motor_entry['topic']

    msg.position = self._saturatedAngle(angle or self.target)
    # Default values are set here, if speed or acceleration are not found in
    # the motor entry.
    msg.speed = self.motor_entry.get('speed', 200)/255.0
    msg.acceleration = self.motor_entry.get('acceleration', 10)/255.0
    return msg

  def msg_intensity(self, intensity=1):
    """Builds a message given the fractional distance (0 to 1) from rest position to target."""
    angle = self._fracDist2val(
      intensity,
      {'min': self.rest, 'max': self.target}
    )
    return self.msg_angle(angle)

  def _saturatedAngle(self, angle):
    return min(max(angle, self.motor_entry['min']), self.motor_entry['max'])

  @staticmethod
  def _saturatedCoeff(coeff):
    return min(max(coeff, 0), 1)

  @staticmethod
  def _fracDist2val(fracDist, interval):
    return (interval['max'] - interval['min']) * fracDist + interval['min']

  def __init__(self, motor_entry, targetFracDist=None):
    # See _fracDist2val() for how targetFracDist relates to the target angle.

    self.motor_entry = motor_entry

    self.target = (
      self._fracDist2val(self._saturatedCoeff(targetFracDist), motor_entry)
      if targetFracDist != None
      else self._saturatedAngle(motor_entry['default'])
    )
    self.rest = motor_entry['default']