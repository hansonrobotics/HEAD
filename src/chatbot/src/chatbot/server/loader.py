import os
import sys
import yaml
import logging
import traceback
from chatbot.server.character import AIMLCharacter, Character
from chatbot.utils import get_location, get_weather
from zipfile import ZipFile

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

    weather = get_weather('{city},{country}'.format(city=location['city'], country=location['country_code']))
    kelvin = 273.15
    if weather and weather['cod'] == 200:
        if 'main' in weather:
            if 'temp_max' in weather['main']:
                dyn_properties['high_temperature'] = '{:.1f}'.format(weather['main'].get('temp_max')-kelvin)
            if 'temp_min' in weather['main']:
                dyn_properties['low_temperature'] = '{:.1f}'.format(weather['main'].get('temp_min')-kelvin)
            if 'temp' in weather['main']:
                dyn_properties['temperature'] = '{:.1f}'.format(weather['main'].get('temp')-kelvin)
        if 'weather' in weather and weather['weather']:
            dyn_properties['weather'] = weather['weather'][0]['description']

load_dyn_properties()

def load_characters(character_path):
    characters = []
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
        errors = []
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
                    errors = character.load_aiml_files(
                        character.kernel, aiml_files)
                if 'weight' in spec:
                    character.weight = float(spec['weight'])
                if 'dynamic_level' in spec:
                    character.dynamic_level = bool(spec['dynamic_level'])
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
            except KeyError as ex:
                logger.error(ex)
                logger.error(traceback.format_exc())
            if errors:
                raise Exception("Loading {} error {}".format(
                    character_yaml, '\n'.join(errors)))
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
    print dyn_properties
