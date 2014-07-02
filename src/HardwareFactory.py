import Utils
import ShapekeyStore
from ros_pololu_servo.msg import servo_pololu
import rospy

class HardwareBase:

  def turn(self, angle):
    return NotImplementedError

  def __init__(self, args):
    """
    On construction parser classes are given the 'args' object (property
    'hardware' of 'binding') parsed from the yaml config file.
    """
    pass

class Pololu(HardwareBase):

  pub = None

  def turn(self, angle):
    self.pub.publish(self.build_msg(angle))

  def build_msg(self, angle):
    msg = servo_pololu()
    msg.id = self.args["id"]
    msg.angle = angle
    msg.speed = self.args["speed"]
    msg.acceleration = self.args["acceleration"]
    return msg

  def __init__(self, args):
    if self.__class__.pub == None:
      self.__class__.pub = rospy.Publisher(
      'cmd_pololu', servo_pololu, queue_size=10
    )

    self.args = args
    
_hardware_classes = {
  "pololu": Pololu
}

def build(yamlobj):
  return _hardware_classes[yamlobj["name"]](yamlobj)