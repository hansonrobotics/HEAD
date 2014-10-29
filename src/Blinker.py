class Blinker:
  def log(self, cmd):
    """
    Log blink-less motor commands of the eye lids so they can be replayed when
    the blink is over.
    """
    if cmd.id in self.names2ids.values():
      self.logged_cmds[cmd.id] = cmd

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

  def __init__(self, robotname, motor_yaml, motornames):
    self.robotname = robotname
    self.names2ids = {
      entry.name: entry.motorid for entry in motor_yaml
      if entry.name in motornames
    }

    cmders = []
    for motorname in motornames:
      cmders.append(MotorCmder(motor_yaml[motorname], 1.0))
    self.motor_commanders = cmders

    self.logged_cmds = {}