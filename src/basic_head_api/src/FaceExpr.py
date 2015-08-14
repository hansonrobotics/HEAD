from collections import OrderedDict
from MotorCmder import MotorCmder
import copy
import rospy

class FaceExprMotors:
  """
  Represents a robot face expression in terms of motor positions. Call msgs()
  to get the corresponding ROS messages for motors.
  """

  def new_msgs(self, intensity=1):
    """
    Returns a list of ROS messages, created from MotorCmder objects, that can be
    published.
    """
    return map(
      lambda motor_cmd: motor_cmd.msg_intensity(intensity),
      self.motor_commanders
    )

  @classmethod
  def from_expr_yaml(cls, expr_yaml, motor_yaml):
    """
    Builds FaceExprMotors instances. Returns a dictionary mapping expression
    names to them.
    """
    result = OrderedDict()
    for exprname in expr_yaml:
      result[exprname] = cls(expr_yaml[exprname], motor_yaml)
    return result

  def __init__(self, expr_entry, motor_yaml):
    # Create MotorCmder objects.
    cmders = []
    for motorname in expr_entry:
      cmders.append(MotorCmder(motor_yaml[motorname], expr_entry[motorname]))
    self.motor_commanders = cmders