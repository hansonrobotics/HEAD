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
    On construction parser classes are given the 'args' object (property
    'parser' of 'binding') parsed from the yaml config file.
    """
    pass

class GetProperty(ParserBase):
  def get_coeff(self, msg):
    coeffs = map(
      lambda kc: kc.get_from(msg),
      self.keychains
    )
    return coeffs if len(coeffs) > 1 else coeffs[0]

  def __init__(self, args):
    self.keychains = map(
      lambda chainstr: Utils.DictKeyChain(chainstr.split(":")),
      args["property"].split(";") 
    )

class FsShapeKey(GetProperty):
  """Construct GetProperty parser from a blendshape name (shapekey)."""

  def __init__(self, args):
    self.keychains = map(
      lambda shapekey: Utils.DictKeyChain([
        "m_coeffs",
        ShapekeyStore.getIndex(shapekey)
      ]),
      args["shapekey"].split(";")
    )
    
_parser_classes = {
  "getproperty": GetProperty,
  "fsshapekey": FsShapeKey
}

def build(yamlobj):
  return _parser_classes[yamlobj["name"]](yamlobj)