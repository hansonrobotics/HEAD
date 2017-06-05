import os
import sys
import yaml
import logging
import traceback
from chatbot.server.character import AIMLCharacter, Character, TYPE_CS, TYPE_AIML
from chatbot.utils import get_location, get_weather, parse_weather
from chatbot.server.config import CS_HOST, CS_PORT
from zipfile import ZipFile
import pprint

logger = logging.getLogger('hr.chatbot.loader')

dyn_properties = {}

def load_dyn_properties():
    global dyn_properties
    location = get_location()
    if location:
        if 'city' in location:
            dyn_properties['location'] = location.get('city')
        elif 'country_name' in location:
            dyn_properties['location'] = location.get('country_name')

    weather_prop = None
    if location:
        weather = get_weather(
            '{city},{country}'.format(
                city=location['city'], country=location['country_code']))
        weather_prop = parse_weather(weather)
    if weather_prop:
        dyn_properties.update(weather_prop)
    logger.info("Update dynamic properties {}".format(dyn_properties))

load_dyn_properties()

def load_characters(character_path):
    characters = []
    if os.path.isfile(character_path) and character_path.endswith('.yaml'):
        characters.extend(ConfigFileLoader.load(character_path))
    else:
        for path in character_path.split(','):
            path = path.strip()
            if not path:
                continue
            sys.path.insert(0, path)
            module_names = [f for f in os.listdir(path) if f.endswith('.py')]
            for module_name in module_names:
                characters.extend(
                    PyModuleCharacterLoader.load(module_name))

            yaml_files = [f for f in os.listdir(path) if f.endswith('.yaml')]
            for yaml_file in yaml_files:
                characters.extend(
                    AIMLCharacterLoader.load(os.path.join(path, yaml_file)))

    for c in characters:
        if c.type == TYPE_CS:
            c.set_host(CS_HOST)
            c.set_port(CS_PORT)

    logger.info("Add characters \n{}".format(pprint.pformat(characters)))
    return characters

class ConfigFileLoader(object):

    @staticmethod
    def load(yaml_file):
        characters = []
        with open(yaml_file) as f:
            config = yaml.load(f)
            if 'tiers' not in config or 'name' not in config or 'properties' not in config:
                logger.error("Wrong format of config file")
                return []
            properties = config['properties']
            for tier in config['tiers']:
                _module, name = tier['type'].rsplit('.', 1)
                try:
                    module  = __import__(_module, fromlist=[name])
                except ImportError as ex:
                    logger.error(ex)
                    continue
                if hasattr(module, name):
                    clazz = getattr(module, name)
                else:
                    logger.error("Module {} has no attribute {}".format(module, name))
                    continue
                try:
                    if 'args' in tier:
                        character = clazz(**tier['args'])
                    else:
                        character = clazz()
                except TypeError as ex:
                    logger.error('Initialize {} error {}'.format(clazz, ex))
                    continue

                if not properties.get('location'):
                    location = dyn_properties.get('location')
                    if location:
                        properties['location'] = location
                if not properties.get('weather'):
                    weather = dyn_properties.get('weather')
                    if weather:
                        properties['weather'] = dyn_properties.get('weather')
                        properties['temperature'] = dyn_properties.get('temperature')
                characters.append(character)

            for c in characters:
                c.set_properties(properties)
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
            logger.error(traceback.format_exc())
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
                errors = []
                root_dir = os.path.dirname(os.path.realpath(character_yaml))
                if 'name' in spec:
                    names = spec['name']
                else:
                    names = 'global'
                for name in names.split(','):
                    name = name.strip()
                    character = AIMLCharacter(spec['id'], name)
                    if 'property_file' in spec:
                        character.set_property_file(abs_path(spec['property_file']))
                    if 'level' in spec:
                        character.level = int(spec['level'])
                    if 'aiml' in spec:
                        aiml_files = [abs_path(f) for f in spec['aiml']]
                        errors = character.load_aiml_files(
                            character.kernel, aiml_files)
                    if 'weight' in spec:
                        character.weight = float(spec['weight'])
                    if 'dynamic_level' in spec:
                        character.dynamic_level = bool(spec['dynamic_level'])
                    if 'non_repeat' in spec:
                        character.non_repeat = bool(spec['non_repeat'])
                    pre_prop = character.get_properties()
                    if not pre_prop.get('location'):
                        location = dyn_properties.get('location')
                        if location:
                            character.set_properties({'location': location})
                    if not pre_prop.get('weather'):
                        weather = dyn_properties.get('weather')
                        if weather:
                            character.set_properties({
                                'weather': dyn_properties.get('weather'),
                                'temperature': dyn_properties.get('temperature'),
                                'low_temperature': dyn_properties.get('low_temperature'),
                                'high_temperature': dyn_properties.get('high_temperature'),
                            })
                    character.print_duplicated_patterns()
                    characters.append(character)
                if errors:
                    raise Exception("Loading {} error {}".format(
                        character_yaml, '\n'.join(errors)))
            except KeyError as ex:
                logger.error(ex)
                logger.error(traceback.format_exc())
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
    print dyn_properties
