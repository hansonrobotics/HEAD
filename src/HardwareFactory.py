import Utils
import ShapekeyStore
from ros_pololu_servo.msg import MotorCommand
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
        self.getTopicName(args["topic"]), self.messageType, queue_size=30
      )
    self.pub = self.__class__._topics[args["topic"]]
    self.args = args

  def getTopicName(self, topic):
      ''' Default topic format '''
      return topic + "/command"

class Pololu(HardwareBase):
  messageType = MotorCommand

  def turn(self, angle):
    self.pub.publish(self.build_msg(angle))


  def build_msg(self, angle):
    msg = MotorCommand()
    msg.joint_name = self.args["name"]
    msg.position = angle
    # IF speed and acceleration is more than 1 default motor speed will be used
    msg.speed = 2
    msg.acceleration = 2
    return msg



class Dynamixel(HardwareBase):

  messageType = Float64

  def turn(self, angle):
    self.pub.publish(angle)

  def getTopicName(self, topic):
      ''' Default topic format '''
      return topic + "_controller/command"


_hardware_classes = {
  "pololu": Pololu,
  "dynamixel": Dynamixel
}

def build(yamlobj):
  return _hardware_classes[yamlobj["hardware"]](yamlobj)
