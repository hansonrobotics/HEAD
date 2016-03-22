import os
from server.character import AIMLCharacter

CWD = os.path.dirname(os.path.realpath(__file__))

sophia = AIMLCharacter('sophia')
sophia.set_property_file(os.path.join(CWD, "../../../character_aiml/sophia.properties"))

han = AIMLCharacter('han')
han.set_property_file(os.path.join(CWD, "../../../character_aiml/han.properties"))

pkd = AIMLCharacter('pkd')
pkd.load_aiml_files([
    os.path.join(CWD, "../../../character_aiml/pkd.*.xml"),
    os.path.join(CWD, "../../../character_aiml/pkd.*.aiml")
])
pkd.set_property_file(os.path.join(CWD, "../../../character_aiml/pkd.properties"))

futurist_sophia = AIMLCharacter('futurist_sophia')
futurist_sophia.load_aiml_files([
    os.path.join(CWD, "../../../futurist_aiml/*.aiml"),
    os.path.join(CWD, "../../../character_aiml/sophia.*.aiml"),
])
futurist_sophia.set_property_file(os.path.join(CWD, "../../../character_aiml/sophia.properties"))

characters = [han, sophia, pkd, futurist_sophia]

