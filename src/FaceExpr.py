from collections import OrderedDict
from MotorCmd import MotorCmd

class FaceExpr:
  """Represents a robot face expression in terms of motor positions."""

  def msgs(self, intensity=1):
    """Returns a list of ROS messages, created from MotorCmd objects, that can be published."""
    return map(
      lambda motor_cmd: motor_cmd.msg_intensity(intensity),
      self.commands
    )

  @staticmethod
  def build_faces(expr_yaml, motor_yaml):
    """Builds FaceExpr instances. Returns a dictionary mapping expression names to them."""
    result = OrderedDict()
    for exprname in expr_yaml:
      result[exprname] = FaceExpr(expr_yaml[exprname], motor_yaml)
    return result

  def __init__(self, expr_entry, motor_yaml):
    # Create MotorCmd objects.
    commands = []
    for motorname in expr_entry:
      commands.append(MotorCmd(motor_yaml[motorname], expr_entry[motorname]))
    self.commands = commands