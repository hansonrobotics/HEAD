import os
from server.character import Character

CWD = os.path.dirname(os.path.realpath(__file__))

sophia = Character('sophia')
sophia.set_property_file(os.path.join(CWD, "../../../character_aiml/sophia.properties"))

han = Character('han')
han.set_property_file(os.path.join(CWD, "../../../character_aiml/han.properties"))

pkd = Character('pkd')
pkd.load_aiml_files([
    os.path.join(CWD, "../../../character_aiml/pkd.*.xml"),
    os.path.join(CWD, "../../../character_aiml/pkd.*.aiml")
])
pkd.set_property_file(os.path.join(CWD, "../../../character_aiml/pkd.properties"))

futurist_sophia = Character('futurist_sophia')
futurist_sophia.load_aiml_files([
    os.path.join(CWD, "../../../character_aiml/sophia.*.xml"),
    os.path.join(CWD, "../../../character_aiml/sophia.*.aiml")
])
futurist_sophia.set_property_file(os.path.join(CWD, "../../../character_aiml/sophia.properties"))

characters = [han, sophia, pkd, futurist_sophia]

