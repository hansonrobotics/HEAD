
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

# Create a dictionary mapping shapekeys to their indices
_shkey2Index = {}
for i in xrange(len(_shkey_list)):
  _shkey2Index[_shkey_list[i]] = i

def getIndex(shapekey):
  """Gets the index of the given shapekey string."""
  return _shkey2Index[shapekey]
