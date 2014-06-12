from MotorCmd import MotorCmd

class EinsteinFace:
  def msgs(self, intensity=1):
    # Return a list of ROS messages to publish created from MotorCmd objects.
    return map(
      lambda motor_cmd: motor_cmd.msg_intensity(intensity),
      self.commands
    )

  @staticmethod
  def build_faces(expr_yaml, motor_yaml):
    """Builds EinsteinFace instances. Returns a dictionary mapping expression names to them."""
    result = {}
    for exprname in expr_yaml:
      result[exprname] = EinsteinFace(expr_yaml[exprname], motor_yaml)
    return result

  def __init__(self, expr_entry, motor_yaml):
    # Create MotorCmd objects.
    commands = []
    for motorname in expr_entry:
      commands.append(MotorCmd(motor_yaml[motorname], expr_entry[motorname]))
    self.commands = commands