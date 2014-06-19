class MapperBase:
  """
  This is an abstract class. Methods 'map' and '__init__' are to be
  implemented when writing a new Mapper class. Subclasses of MapperBase will
  be used as mapping functions that take values received in messages and
  return angles in radians.
  """

  def map(self, val):
    raise NotImplementedError

  def __init__(self, args, motor_entry):
    """
    On construction mapper classes are given the 'args' object (property of
    'binding') parsed from the yaml config file.

    And the whole 'motor_entry' (parent of 'args') mainly to reach 'min' and
    'max' properties.
    """
    pass

class Linear(MapperBase):

  def map(self, val):
    return (val + self.pretranslate) * self.scale + self.posttranslate

  def __init__(self, args, motor_entry):
    if (args.has_key('scale') and args.has_key('translate')):
      self.pretranslate = 0
      self.scale = args['scale']
      self.posttranslate = args['translate']

    elif (args.has_key('min') and args.has_key('max'):
      # Map the given 'min' and 'max' to the motor's 'min' and 'max'
      self.pretranslate = -args['min']
      self.scale = (motor_entry['max']-motor_entry['min'])/(args['max']-args['min'])
      self.posttranslate = motor_entry['min']

_mapper_classes = {
  "linear": Linear
}

def build(name, motor_entry):
  return _mapper_classes[name](args, motor_entry)