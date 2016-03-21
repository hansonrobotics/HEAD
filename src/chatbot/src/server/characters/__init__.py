import os
import sys
from server.character import Character

CWD = os.path.dirname(os.path.realpath(__file__))
DEFAULT_CHARACTER_PATH=CWD
CHARACTER_PATH=os.environ.get('HR_CHARACTER_PATH', DEFAULT_CHARACTER_PATH)
CHARACTERS = []
for path in CHARACTER_PATH.split(':'):
    sys.path.insert(0, path)
    module_names = [f for f in os.listdir(path) if f.endswith('.py')]
    module_names.remove('__init__.py')
    for module_name in module_names:
        module = __import__(module_name[:-3])
        characters = getattr(module, 'characters')
        for character in characters:
            if isinstance(character, Character):
                CHARACTERS.append(character)
                print "Add character {}".format(character)
