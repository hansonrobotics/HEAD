from MotorCmder import MotorCmder

class Pau2Motors:

  def to_motor_cmds(self, pau_msg):
    return map(
      lambda cmder: cmder.build_cmd(pau_msg),
      self.motor_commanders
    )

  def __init__(self, motors_yaml):
    motor_commanders = []

    for motor_entry in motors_yaml:
      motor_commanders.append(MotorCmder(motor_entry))
    self.motor_commanders = motor_commanders
