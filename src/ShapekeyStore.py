__doc__ = """
The lists of shapekeys defined here correspond to the coefficient array
"m_coeffs" in pau.msg. These keys are used in the config files to specify
which coefficients to extract from the incoming PAU messages.
"""

_shkey_list = ['Basis', 'adjustments', 'brow_center_UP', 'brow_inner_UP.L', 'brow_inner_UP.R', 'brow_outer_UP.L',
                   'brow_outer_up.R', 'brow_center_DN', 'brow_inner_DN.L', 'brow_inner_DN.R', 'brow_outer_DN.L',
                   'brow_outer_DN.R', 'eye-blink.UP.L', 'eye-blink.LO.L', 'eye-blink.UP.R', 'eye-blink.LO.R',
                   'eye-flare.UP.L', 'eye-flare.LO.L', 'eye-flare.UP.R', 'eye-flare.LO.R', 'eyes-look.dn',
                   'eyes-look.up', 'wince.L', 'wince.R', 'sneer.L', 'sneer.R', 'lips-wide.L', 'lips-wide.R',
                   'lips-narrow.L', 'lips-narrow.R', 'lips-frown.L', 'lips-frown.R', 'lips-smile.L', 'lips-smile.R',
                   'lip-UP.C.UP', 'lip-UP.C.DN', 'lip-UP.L.UP', 'lip-UP.L.DN', 'lip-UP.R.UP', 'lip-UP.R.DN',
                   'lip-DN.C.UP', 'lip-DN.C.DN', 'lip-DN.L.UP', 'lip-DN.L.DN', 'lip.DN.R.UP', 'lip-DN.R.DN',
                   'lip-JAW.DN', 'jaw']


def _build_index(lst):
  """Build a dictionary mapping the given list values to their indices"""
  result = {}
  for i in xrange(len(lst)):
    result[lst[i]] = i
  return result

#Call _build_index() for every shkey_list
_shkey2index = [
  _build_index(shkey_list)
  for shkey_list in [_shkey_list]
]

_current_dict = _shkey2index[0]

def _get_dict_with(shapekey):
  """
  Gets the shkey2index dictionary, which has the given shapekey in it out of
  available shkey2index dictionaries.
  """
  result = None
  for dct in _shkey2index:
    if shapekey in dct:
      result = dct
      break
  if result == None:
    raise KeyError("Shapekey '%s' not found." % shapekey)
  return result

def getIndex(shapekey):
  """Gets the index of the given shapekey string."""
  global _current_dict
  if not shapekey in _current_dict:
    _current_dict = _get_dict_with(shapekey)
  return _current_dict[shapekey]
