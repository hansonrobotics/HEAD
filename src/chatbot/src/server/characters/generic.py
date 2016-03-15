import os
from server.character import Character

CWD = os.path.dirname(os.path.realpath(__file__))
generic = Character('generic')
generic.load_aiml_files([
    os.path.join(CWD, "../../../aiml/*.aiml"),
])

characters = [generic]
