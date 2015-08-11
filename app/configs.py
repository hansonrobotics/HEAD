'''
Creates confg from motor settings page
'''
import copy
import math
import json

class ConfigError(Exception):
    pass

class ConfigWarning(Exception):
    pass

class Configs:

    # Templates for configs
    _MOTORS_TPL={
        'name': '',
        'topic': '',
        'labelleft': '',
        'default': 0,
        'min': None,
        'max': None,
        'sort_no': 0,
        'group': 'Ungrouped',
        'hardware': None
    }

    _PAU_TPL = {
        'parser':{
            'name': None
        },
        'function': [
        ]
    }

    _PAU_FUNC_WEIGHTEDSUM_TPL = {
        'name': 'weightedsum',
        'imin': None,
        'terms': [
            {'min': 0, 'max': 1, 'imax': 0},
            {'min': 0, 'max': 1, 'imax': 1},
        ],
    }

    _PAU_FUNC_LINEAR_TPL = {
        'name': 'linear',
        'min': None,
        'max': None
    }

    _DYNAMIXELS_TPL = {
        'controller': {
            'package': 'dynamixel_controllers',
            'module': 'joint_position_controleller',
            'type': 'JointPositionController',
        },
        'joint_name': None,
        'joint_speed': 1,
        'motor' : {
            'id': None,
            'init': None,
            'min': None,
            'max': None,
            'acceleration': '20',
        }
    }
    _POLOLU_TPL = {
        'motor_id': None,
        'init': None,
        'min': None,
        'max': None,
        'labelleft': '',
        'sort_no': 0,
        'group': 'Default',
        'spped': 0,
        'acceleration': 0,
    }
    def __init__(self):
        # Multiple configs for pololu boards
        self.pololu = {}
        # Dynamixel manager config.
        self.dynamixel = {}
        # Motors config for other nodes to use. Currently dynamixels only included but pololu callibration will be moved.
        self.motors = []

    # Parse motors
    def parseMotors(self,motors):
        for i, m in enumerate(motors):
            self._add_motor(m)
            self._add_pololu(m)
            self._add_dynamixel(m)

    # Makes the entry for motors config
    def _add_motor(self,m):
        # Only dynamixels currently
        if m['hardware'] != 'dynamixel':
            return
        c = copy.deepcopy(self._MOTORS_TPL)
        # Copy values to template
        c['name'] = m['name']
        c['topic'] = m['name']
        c['labelleft'] = m['name']
        c['min'] = self.dynamixel_angle(m, m['min'])
        c['max'] = self.dynamixel_angle(m, m['max'])
        c['sort_no'] = m['sort_no']
        c['group'] = m['group']
        c['hardware'] = m['hardware']
        pau = self._get_pau(m)
        if pau:
            c['pau'] = pau
        self.motors.append(c)

    def _get_pau(self,m):
        p = copy.deepcopy(self._PAU_TPL)
        # Parsers
        p['parser']['name'] = m['parser']
        if m['parser'] == 'getproperty':
            p['parser']['property'] = m['parser_param']
        if m['parser'] == 'fsshapekey':
            p['parser']['shapekey'] = m['parser_param']
        # mapping function
        # Linear function
        if m['function'] == 'linear':
            p['function'].append(copy.deepcopy(self._PAU_FUNC_LINEAR_TPL))
            p['function'][0]['min'] = m['lin_min']
            p['function'][0]['max'] = m['lin_max']

        # Weighted sum
        if m['function'] == 'weightedsum':
            p['function'].append(copy.deepcopy(self._PAU_FUNC_WEIGHTEDSUM_TPL))
            p['function'][0]['imin'] = self.imin(m)
            p['function'][0]['terms'][0]['max'] = m['max1']
            p['function'][0]['terms'][0]['imax'] = m['imax1']
            p['function'][0]['terms'][1]['max'] = m['max2']
            p['function'][0]['terms'][1]['imax'] = m['imax2']
        # Other additional functions can be added
        if m['other_func']:
            other_func = json.load(m['other_func'])
            if isinstance(other_func, list):
                p['function'] += other_func
            else:
                p['function'].append(other_func)
        return p

    def imin(self, m):
        return (m['init'] - m['min']) / float(m['max'] - m['min'])

    def dynamixel_angle(self, m, v):
        return (v - m['init']) * math.pi / 2048

    def _add_dynamixel(self,m):
        if m['hardware'] != 'dynamixel':
            return

        c = copy.deepcopy(self._DYNAMIXELS_TPL)
        c['joint_name'] = m['name'] + '_joint'
        c['speed'] = m['speed']
        c['motor']['id'] = m['motor_id']
        c['motor']['init'] = m['init']
        c['motor']['min'] = m['min']
        c['motor']['max'] = m['max']
        c['motor']['acceleration'] = m['acceleration']
        name = m['name'] +'_controller'
        self.dynamixel[name] = c

    def _add_pololu(self,m):
        # Only pololu currently
        if m['hardware'] != 'pololu':
            return
        c = copy.deepcopy(self._POLOLU_TPL)
        # Copy values to template
        c['labelleft'] = m['name']
        c['min'] = m['min']
        c['max'] = m['max']
        c['init'] = m['init']
        c['sort_no'] = m['sort_no']
        c['group'] = m['group']
        c['speed'] = m['speed']
        c['acceleration'] = m['acceleration']

        pau = self._get_pau(m)
        if pau:
            c['pau'] = pau
        self.pololu[m['name']] = c









