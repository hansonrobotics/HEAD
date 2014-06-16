class MapperBase:
  """
  This is an abstract class. Methods 'map' and '__init__' are to be
  implemented when writing a new Mapper class. Subclasses of MapperBase will
  be used as mapping functions that take values received in messages and
  return angles in radians.
  """

  def map(self, val):
    raise NotImplementedError

  def __init__(self, args):
    """
    On construction mapper classes are given the 'args' object (property of
    'binding') parsed from the yaml config file.
    """
    pass

class Linear(MapperBase):
  def map(self, val):
    return val * self.args['scale'] + self.args['translate']

  def __init__(self, args):
    self.args = args
    pass

_mapper_classes = {
  "linear": Linear
}

def build(name, args):
  return _mapper_classes[name](args)