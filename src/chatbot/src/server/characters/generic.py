import os
from server.character import AIMLCharacter

CWD = os.path.dirname(os.path.realpath(__file__))
generic = AIMLCharacter('generic', 'generic')
generic.load_aiml_files([
    os.path.join(CWD, "../../../aiml/*.aiml"),
])

characters = [generic]
