import os
import yaml
import logging
from server.character import AIMLCharacter

CWD = os.path.dirname(os.path.realpath(__file__))

logger = logging.getLogger('hr.chatbot.characters.aiml_characters')

def load_aiml_character(character_yaml):
    def abs_path(p):
        if p.startswith('/'):
            return p
        return os.path.join(root_dir, p)

    with open(character_yaml) as f:
        spec = yaml.load(f)
        try:
            root_dir = os.path.dirname(os.path.realpath(character_yaml))
            character = AIMLCharacter(spec['name'])
            character.set_property_file(abs_path(spec['property_file']))
            aiml_files = [abs_path(f) for f in spec['aiml']]
            character.load_aiml_files(aiml_files)
        except KeyError as ex:
            logger.error(ex)

    return character

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

characters = [han, sophia, pkd]

characters.append(load_aiml_character(os.path.join(CWD, 'quant_han.yaml')))
