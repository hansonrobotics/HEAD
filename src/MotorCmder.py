from ros_pololu_servo.msg import servo_pololu
import ParserFactory
import MapperFactory
import rospy

class MotorCmder:
  """
  MotorCmder (or Motor Commander) represents a partially defined motor command.
  It builds a ROS motor message when given a message that the specified parser
  expects.
  """

  def build_cmd(self, incoming_msg):
    """Builds a motor command given a PAU message."""

    coeff = self.parser.get_coeff(incoming_msg)
    angle = self.mapper.map(coeff)
    rospy.logdebug("Motor: %s, coeff: %s, angle: %s", 
      self.motor_entry['name'], coeff, angle
    )

    msg = servo_pololu()
    msg.id = self.motor_entry['motorid']
    msg.angle = self._saturated(angle)
    msg.speed = self.motor_entry['speed']
    msg.acceleration = self.motor_entry['acceleration']

    return msg

  def _saturated(self, angle):
    return min(max(angle, self.motor_entry['min']), self.motor_entry['max'])  

  def __init__(self, motor_entry):
    binding_obj = motor_entry["binding"]

    self.parser = ParserFactory.build(
      binding_obj["parser"],
      binding_obj["args"]
    )
    self.mapper = MapperFactory.build(
      binding_obj["function"],
      binding_obj["args"]
    )
    self.motor_entry = motor_entry
