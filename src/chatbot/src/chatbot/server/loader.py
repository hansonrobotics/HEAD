import os
import sys
import yaml
import logging
from server.character import AIMLCharacter, Character
from zipfile import ZipFile

logger = logging.getLogger('hr.chatbot.loader')

def load_characters(character_path):
    characters = []
    for path in character_path.split(','):
        path = path.strip()
        if not path: continue
        sys.path.insert(0, path)
        module_names = [f for f in os.listdir(path) if f.endswith('.py')]
        for module_name in module_names:
            characters.extend(
                PyModuleCharacterLoader.load(module_name))

        yaml_files = [f for f in os.listdir(path) if f.endswith('.yaml')]
        for yaml_file in yaml_files:
            characters.extend(
                AIMLCharacterLoader.load(os.path.join(path, yaml_file)))

    logger.info("Add characters {}".format(characters))
    return characters

class PyModuleCharacterLoader(object):

    @staticmethod
    def load(py_module):
        characters = []
        try:
            module = __import__(py_module[:-3])
            if hasattr(module, 'characters'):
                for character in getattr(module, 'characters'):
                    if isinstance(character, Character):
                        characters.append(character)
        except Exception as ex:
            logger.error(ex)
        return characters

class AIMLCharacterLoader(object):

    @staticmethod
    def load(character_yaml):
        def abs_path(p):
            if p.startswith('/'):
                return p
            if p.startswith('~'):
                return os.path.expanduser(p)
            return os.path.join(root_dir, p)

        characters = []
        with open(character_yaml) as f:
            spec = yaml.load(f)
            try:
                root_dir = os.path.dirname(os.path.realpath(character_yaml))
                character = AIMLCharacter(spec['id'], spec['name'])
                if 'property_file' in spec:
                    character.set_property_file(abs_path(spec['property_file']))
                if 'level' in spec:
                    character.level = int(spec['level'])
                if 'aiml' in spec:
                    aiml_files = [abs_path(f) for f in spec['aiml']]
                    character.load_aiml_files(character.kernel, aiml_files)
                if 'weight' in spec:
                    character.weight = float(spec['weight'])
            except KeyError as ex:
                logger.error(ex)
            characters.append(character)
        return characters

from gsheet_chatter import batch_csv2aiml
class AIMLCharacterZipLoader(object):

    @staticmethod
    def load(zipfile, output_dir, dirname):
        """
        output_dir: directory the zip file is decompressed to. 
        dirname: the name of the top level directory in the zip file.
        """
        with ZipFile(zipfile) as f:
            f.extractall(output_dir)

        characters = []
        dirpath = os.path.join(output_dir, dirname)
        batch_csv2aiml(dirpath, dirpath)
        for yaml_file in os.listdir(dirpath):
            if yaml_file.endswith('.yaml'):
                characters.extend(AIMLCharacterLoader.load(
                                os.path.join(dirpath, yaml_file)))
        return characters

if __name__ == '__main__':
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)
    characters = load_characters('characters')
    print characters
