from threading import Timer
import random
from MotorCmder import MotorCmder

class Blinker:
  def log(self, cmd):
    """
    Log blink-less motor commands of the eye lids so they can be replayed when
    the blink is over.
    """
    if cmd.joint_name in self.names2ids.values():
      self.logged_cmds[cmd.joint_name] = cmd

  def new_msgs(self, intensity=1):
    """
    Return messages ready for publishing.
    intensity=1 to close eyes, intensity=0 to open eyes
    """
    return [cmd.msg_intensity(intensity) for cmd in self.motor_commanders]

  def reset_msgs(self):
    """
    Return the last eye lid motor commands sent through the original channels.
    """
    return self.logged_cmds.values()

  def add_motor(self, motorname, frac_dist):
    self.names2ids[motorname] = motorname

    self.motor_commanders.append(
      MotorCmder(
        self.motor_yaml[motorname],
        frac_dist
      )
    )


  def __init__(self, robotname, motor_yaml):
    self.robotname = robotname
    self.motor_yaml = motor_yaml
    self.names2ids = {}
    self.motor_commanders = []
    self.logged_cmds = {}

class RandomTimer:

  is_running = False
  timer = None

  def _hit(self):
    if self.timer:
      self.timer.cancel()
    if self.is_running:
      self.func()
      self.timer = Timer(random.gauss(*self.mu_sig), self._hit)
      self.timer.start()

  def start(self):
    self.is_running = True
    self._hit()

  def stop(self):
    self.is_running = False

  def __init__(self, func, mu_sig):
    self.func = func
    self.mu_sig = mu_sig