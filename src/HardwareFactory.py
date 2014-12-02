import Utils
import ShapekeyStore
from ros_pololu_servo.msg import servo_pololu
from std_msgs.msg import Float64
import rospy

class HardwareBase:

  _topics = {}
  pub = None

  messageType = None

  def turn(self, angle):
    return NotImplementedError

  def getMessageType(self):
      return  self.messageType

  def __init__(self, args):
    ''' On construction parser classes are given the 'args' object (property
    'hardware' of 'binding') parsed from the robot config file in robots_config. '''

    if not self.__class__._topics.has_key(args["topic"]):
      self.__class__._topics[args["topic"]] = rospy.Publisher(
        args["topic"], self.messageType, queue_size=30
      )
    self.pub = self.__class__._topics[args["topic"]]
    self.args = args


class Pololu(HardwareBase):
  messageType = servo_pololu

  def turn(self, angle):
    self.pub.publish(self.build_msg(angle))


  def build_msg(self, angle):
    msg = servo_pololu()
    msg.id = self.args["id"]
    msg.angle = angle
    msg.speed = self.args["speed"]
    msg.acceleration = self.args["acceleration"]
    return msg



class Dynamixel(HardwareBase):

  messageType = Float64

  def turn(self, angle):
    self.pub.publish(angle)


_hardware_classes = {
  "pololu": Pololu,
  "dynamixel": Dynamixel
}

def build(yamlobj):
  return _hardware_classes[yamlobj["name"]](yamlobj)