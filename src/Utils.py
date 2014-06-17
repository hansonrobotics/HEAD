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