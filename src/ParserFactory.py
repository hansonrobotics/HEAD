import Utils

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
    return self.keychain.get_value(msg)

  def __init__(self, args):
    self.keychain = Utils.DictKeyChain(
      args["property"].split(":")
    )

class FsShapeKey(GetProperty):
  """Construct GetProperty parser from a blendshape name (shapekey)."""

  # Will hold a dictionary mapping 48 blendshape names (shapekeys) used by
  # faceshift to their indices.
  _shkey2index = None

  @classmethod
  def _load_shapekeys(cls):
    """
    Fills _shkey2index.
    Called when constructing the first instance of FSMsgParser.
    """
    shkey_list = Utils.read_yaml("fs_shapekeys.yaml")
    d = {}
    for i in xrange(len(shkey_list)):
      d[shkey_list[i]] = i
    cls._shkey2index = d

  def __init__(self, args):
    if not self._shkey2index:
      self._load_shapekeys()

    self.keychain = Utils.DictKeyChain([
      "m_coeffs", 
      self._shkey2index[args['shapekey']]
    ])
    
_parser_classes = {
  "getproperty": GetProperty,
  "fsshapekey": FsShapeKey
}

def build(name, args):
  return _parser_classes[name](args)