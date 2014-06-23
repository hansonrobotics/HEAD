from collections import OrderedDict
from MotorCmder import MotorCmder
from pau2motors.msg import fsMsgTrackingState
import ShapekeyStore
import copy
import rospy

class FaceExprPAU:
  """
  Represents a robot face expression in terms of PAU coefficients.
  Call msg() to get the corresponding ROS message.
  """

  M_COEFFS_LEN = 48

  def new_msg(self, intensity=1):
    """
    Builds a fsMsgTrackingState message based on the given expression
    intensity (0..1).
    """
    result = copy.deepcopy(self.msg)
    for i in xrange(self.M_COEFFS_LEN):
      result.m_coeffs[i] *= intensity
    return result

  @classmethod
  def from_expr_yaml(cls, expr_yaml):
    """
    Builds FaceExprPAU instances from config file. Returns a dictionary
    mapping expression names to them.
    """
    result = OrderedDict()
    for exprname in expr_yaml:
      result[exprname] = cls(expr_yaml[exprname])
    return result

  @classmethod
  def _build_msg(cls, expr_entry):
    """
    Takes an entry from the config file and returns a constructed ROS message.
    """
    msg = fsMsgTrackingState()
    msg.m_coeffs = [0]*cls.M_COEFFS_LEN
    for shapekey in expr_entry:
      index = ShapekeyStore.getIndex(shapekey)
      if (index == None):
        rospy.logwarn("Invalid shapekey in expression data: %s", shapekey)
      else:
        msg.m_coeffs[index] = expr_entry[shapekey]
    return msg

  def __init__(self, expr_entry):
    self.msg = self._build_msg(expr_entry)
  

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