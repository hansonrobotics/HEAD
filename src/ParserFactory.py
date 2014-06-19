import Utils
import ShapekeyStore

class ParserBase:
  """
  Abstract class. Methods 'get_value' and '__init__' are to be implemented.
  """

  def get_coeff(self, msg):
    """
    Implementations of this method should extract the appropriate value or
    values in PAU representation from the provided 'msg' object. The returned
    value will be sent to a mapper class.
    """
    return NotImplementedError

  def __init__(self, args):
    """
    On construction parser classes are given the 'args' object (property of
    'binding') parsed from the yaml config file.
    """
    pass

class GetProperty(ParserBase):
  def get_coeff(self, msg):
    return self.keychain.get_from(msg)

  def __init__(self, args):
    self.keychain = Utils.DictKeyChain(
      args["property"].split(":")
    )

class FsShapeKey(GetProperty):
  """Construct GetProperty parser from a blendshape name (shapekey)."""

  def __init__(self, args):
    self.keychain = Utils.DictKeyChain([
      "m_coeffs",
      ShapekeyStore.getIndex(args["shapekey"])
    ])
    
_parser_classes = {
  "getproperty": GetProperty,
  "fsshapekey": FsShapeKey
}

def build(name, args):
  return _parser_classes[name](args)