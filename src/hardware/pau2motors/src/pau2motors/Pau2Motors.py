from MotorCmder import MotorCmder

class Pau2Motors:

  def consume(self, pau_msg):
    for cmder in self.motor_commanders:
      cmder.consume(pau_msg)

  def __init__(self, motors_yaml):
    motor_commanders = []

    for motor_entry in motors_yaml:
      motor_commanders.append(MotorCmder(motor_entry))
    self.motor_commanders = motor_commanders
