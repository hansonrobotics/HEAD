import os
import yaml

class DictKeyChain:
  """
  Allows to build a chain of dictionary keys (or array indices or object
  attributes), which can be later applied to an obj.

  I.e. a DictKeyChain instance constructed with key_list ['foo', 'bar'], can
  be applied on a dictionary {'foo': {'bar': 'myStr'}} to get the 'myStr'
  string.
  """

  def get_from(self, host):
    for key in self.key_list:
      if hasattr(host, "__getitem__"):
        host = host[key]
      else:
        host = getattr(host, key)
    return host

  def __init__(self, key_list):
    # Parse strings to integers where possible
    self.key_list = map(
      lambda key: int(key)
        if isinstance(key, str) and key.isdigit()
        else key,
      key_list
    )


def read_yaml(yamlname):
  dirname, filename = os.path.split(os.path.abspath(__file__))

  stream = open(os.path.join(dirname, yamlname), 'r')
  result = yaml.load(stream)
  stream.close()
  return result

def append_dictlist(dictlist, key, value):
  """
  Appends the list in the given dict at the given key with the value.
  Creates a new list at that place if needed.
  """
  if (dictlist.has_key(key)):
    dictlist[key].append(value)
  else:
    dictlist[key] = [value]

def list2dictlist(mylist, keychain, mapfunc=None):
  """
  Returns a dictionary that maps values found at the given 'keychain' of
  entries in 'mylist' to lists of corresponding entries and optionally passes
  them through the given 'mapfunc'.

  E.g. 
  list2dictlist(
    [{'k':5}, {'k':6, 'l':7}],
     DictKeyChain(['k'])
  )
  would return {5: [{'k:5'}, 6: {'k':6, 'l':7}]}

  list2dictlist(
    [{'k':5}, {'k':6, 'l':7}],
    DictKeyChain(['k']),
    lambda x: len(x)
  )
  would return {5:1, 6:2}
  """

  for entry in mylist:
    append_dictlist(
      dictlist,
      keychain.get_from(entry),
      entry if mapfunc == None else mapfunc(entry)
    )
  return dictlist
