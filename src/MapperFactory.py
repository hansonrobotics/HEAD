import math

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
    On construction mapper classes are given the 'args' object (property
    'function' of 'binding') parsed from the yaml config file.

    And the whole 'motor_entry' (parent of 'binding') mainly to reach 'min'
    and 'max' properties.
    """
    pass

class Composite(MapperBase):

  def map(self, val):
    for mapper_instance in self.mapper_list:
      val = mapper_instance.map(val)
    return val

  def __init__(self, mapper_list):
    self.mapper_list = mapper_list

class Linear(MapperBase):

  def map(self, val):
    return (val + self.pretranslate) * self.scale + self.posttranslate

  def __init__(self, args, motor_entry):
    if args.has_key('scale') and args.has_key('translate'):
      self.pretranslate = 0
      self.scale = args['scale']
      self.posttranslate = args['translate']

    elif args.has_key('min') and args.has_key('max'):
      # Map the given 'min' and 'max' to the motor's 'min' and 'max'
      self.pretranslate = -args['min']
      self.scale = (motor_entry['max']-motor_entry['min'])/(args['max']-args['min'])
      self.posttranslate = motor_entry['min']

class WeightedSum(MapperBase):
  """
  This will map min-max range for every term to intermediate min-max range
  (imin-imax) and sums them up. Then the intermediate 0..1 range will be
  mapped to the motor min-max range.
  """

  @staticmethod
  def _saturated(val, interval):
    return min(max(val, interval["min"]), interval["max"])

  def map(self, vals):
    return sum(
      map(
        lambda (val, translate, scale, term): 
          (self._saturated(val, term) + translate) * scale,
        zip(vals, self.pretranslations, self.scalefactors, self.termargs) 
      )
    ) + self.posttranslate

  def __init__(self, args, motor_entry):
    range_motors = motor_entry["max"] - motor_entry["min"]

    self.posttranslate = args["imin"] * range_motors + motor_entry["min"]
    self.pretranslations = map(
      lambda term: -term["min"],
      args["terms"]
    )
    self.scalefactors = map(
      lambda term:
        (term["imax"]-args["imin"])/(term["max"]-term["min"])*range_motors,
      args["terms"]
    )
    self.termargs = args["terms"]

class Quaternion2Euler(MapperBase):

  def __init__(self, args, motor_entry):
    funcsByAxis = {
      'x': lambda q: math.atan2(
        -2*(q['y']*q['z']-q['w']*q['x']),
        q['w']*q['w']-q['x']*q['x']-q['y']*q['y']+q['z']*q['z']
      ),
      'y': lambda q: math.asin(
        2*(q['x']*q['z'] + q['w']*q['y'])
      ),
      'z': lambda q: math.atan2(
        -2*(q['x']*q['y']-q['w']*q['z']),
        q['w']*q['w']+q['x']*q['x']-q['y']*q['y']-q['z']*q['z']
      )
    }
    self.map = funcsByAxis[args['axis'].lower()]

_mapper_classes = {
  "linear": Linear,
  "weightedsum": WeightedSum,
  "quaternion2euler": Quaternion2Euler
}

def build(yamlobj, motor_entry):
  if isinstance(yamlobj, dict):
    return _mapper_classes[yamlobj["name"]](yamlobj, motor_entry)
  elif isinstance(yamlobj, list):
    return Composite(
      [build(func_entry, motor_entry) for func_entry in yamlobj]
    )
