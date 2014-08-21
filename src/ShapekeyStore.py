
_shkey_list = [
  "EyeBlink_L",
  "EyeBlink_R",
  "EyeSquint_L",
  "EyeSquint_R",
  "EyeDown_L",
  "EyeDown_R",
  "EyeIn_L",
  "EyeIn_R",
  "EyeOpen_L",
  "EyeOpen_R",
  "EyeOut_L",
  "EyeOut_R",
  "EyeUp_L",
  "EyeUp_R",
  "BrowsD_L",
  "BrowsD_R",
  "BrowsU_C",
  "BrowsU_L",
  "BrowsU_R",
  "JawFwd",
  "JawLeft",
  "JawOpen",
  "JawChew",
  "JawRight",
  "MouthLeft",
  "MouthRight",
  "MouthFrown_L",
  "MouthFrown_R",
  "MouthSmile_L",
  "MouthSmile_R",
  "MouthDimple_L",
  "MouthDimple_R",
  "LipsStretch_L",
  "LipsStretch_R",
  "LipsUpperClose",
  "LipsLowerClose",
  "LipsUpperUp",
  "LipsLowerDown",
  "LipsUpperOpen",
  "LipsLowerOpen",
  "LipsFunnel",
  "LipsPucker",
  "ChinLowerRaise",
  "ChinUpperRaise",
  "Sneer",
  "Puff",
  "CheekSquint_L",
  "CheekSquint_R"
]

_shkey_list2 = [ 
  "00_EyeBlink_L",
  "01_EyeBlink_R",
  "02_EyeSquint_L",
  "03_EyeSquint_R",
  "04_EyeDown_L",
  "05_EyeDown_R",
  "08_EyeOpen_L",
  "09_EyeOpen_R",
  "12_EyeUp_L",
  "13_EyeUp_R",
  "14_BrowsD_I_L",
  "14_BrowsD_O_L",
  "15_BrowsD_O_R",
  "15_BrowsD_I_R",
  "16_BrowsU_C",
  "17_BrowsU_O_L",
  "17_BrowsU_I_L",
  "18_BrowsU_I_R",
  "18_BrowsU_O_R",
  "21_JawOpen",
  "26_MouthFrown_L",
  "27_MouthFrown_R",
  "28_MouthSmile_L",
  "29_MouthSmile_R",
  "32_LipsStretch_L",
  "33_LipsStretch_R",
  "36_LipsUpperUp_L",
  "36_LipsUpperUp_R",
  "37_LipsLowerDown_L",
  "37_LipsLowerDown_R",
  "38_LipsUpperOpen",
  "39_LipsLowerOpen",
  "40_LipsFunnel",
  "41_LipsPucker",
  "42_ChinLowerRaise",
  "43_ChinUpperRaise",
  "44_Sneer_C",
  "44_Sneer_L",
  "44_Sneer_R",
  "46_CheekSquint_L",
  "47_CheekSquint_R"
]

def _build_index(lst):
  """Build a dictionary mapping the given list values to their indices"""
  result = {}
  for i in xrange(len(lst)):
    result[lst[i]] = i
  return result

#Call _build_index() for every shkey_list
_shkey2index = [
  _build_index(shkey_list)
  for shkey_list in [_shkey_list, _shkey_list2]
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
