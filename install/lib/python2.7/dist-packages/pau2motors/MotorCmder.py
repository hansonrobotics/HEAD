import ParserFactory
import MapperFactory
import HardwareFactory
import rospy
import logging

logger = logging.getLogger('hr.pau2motors.motorcmder')

class MotorCmder:
  """
  MotorCmder (or Motor Commander) is a three-stage adapter put on top of a motor
  and takes PAU messages to control it.
  """

  def consume(self, incoming_msg):
    coeff = self.parser.get_coeff(incoming_msg)
    angle = self.mapper.map(coeff)
    logger.debug("Motor: %s, coeff: %s, angle: %s",
                self.motor_entry['name'], coeff, angle)
    self.hardware.turn(self._saturated(angle))

  def _saturated(self, angle):
    return min(max(angle, self.motor_entry['min']), self.motor_entry['max'])  

  def __init__(self, motor_entry):

    binding_obj = motor_entry["pau"]

    self.parser = ParserFactory.build(
      binding_obj["parser"]
    )
    self.mapper = MapperFactory.build(
      binding_obj["function"],
      motor_entry
    )
    self.hardware = HardwareFactory.build(
        motor_entry
    )
    self.motor_entry = motor_entry
