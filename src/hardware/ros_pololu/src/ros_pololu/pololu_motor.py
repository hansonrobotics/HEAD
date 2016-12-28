import math


class ConfigError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class PololuMotor:
    def __init__(self, name, config):
        """
        :param name:
            Motor name
        :param config:
            motor configuration. Requires motor_id, min, max, init to be setup. Others can be left for default values.
        """
        self.name = name
        self._config = config
        if 'init' not in config.keys():
            raise ConfigError("Init value missing for {0}".format(name))
        if 'min' not in config.keys():
            raise ConfigError("Min value missing for {0}".format(name))
        if 'max' not in config.keys():
            raise ConfigError("Max value missing for {0}".format(name))
        if 'motor_id' not in config.keys():
            raise ConfigError("motor_id value missing for {0}".format(name))
        # pulses 1/4 of  micro s
        self.pulse = int(self._config['init'] * 4)
        self._setup_calibration()
        self.id = self._config['motor_id']
        # Store speed and acceleration. By default should be small values to slowly turn tu neutral on start
        self.speed = 0.3
        self.acceleration = 0.1
        if 'speed' in config.keys():
            self.speed = self._config['speed']
        if 'acceleration' in config.keys():
            self.acceleration = self._config['acceleration']
        self.last_pulse = self.pulse

    def _setup_calibration(self):
        """
        Initializes motor calibration values. Sets the min and max angles in radians
        """
        self._calibration = {
            'min_angle': 0.0,
            'max_angle': 0.0,
            'min_pulse': self._config['min'] * 4,
            'max_pulse': self._config['max'] * 4,
        }
        # Calibration provided
        if 'calibration' in self._config.keys():
            c = self._config['calibration']
            pa = (c['max_angle'] - c['min_angle']) / float((c['max_pulse'] - c['min_pulse']))
            self._calibration['min_angle'] = math.radians(c['min_angle'] + (self._config['min'] - c['min_pulse']) * pa)
            self._calibration['max_angle'] = math.radians(c['max_angle'] + (self._config['max'] - c['max_pulse']) * pa)
        else:
            # set init position to 0 with range of 90 degrees
            pa = 90 / float((self._config['max'] - self._config['min']))
            self._calibration['min_angle'] = math.radians((self._config['min'] - self._config['init']) * pa)
            self._calibration['max_angle'] = math.radians((self._config['max'] - self._config['init']) * pa)

    def _angle_to_pulse(self, angle):
        """
        :param angle:  Angle in radians
        :return: pulse
        """
        pa = (self._calibration['min_pulse'] - self._calibration['max_pulse']) /\
             (self._calibration['min_angle'] - self._calibration['max_angle'])
        return self._calibration['min_pulse'] + (angle - self._calibration['min_angle']) * pa

    def set_angle(self, angle):
        # Check the val
        if angle < min(self._calibration['min_angle'], self._calibration['max_angle']) or \
                angle > max(self._calibration['min_angle'], self._calibration['max_angle']):
            raise ConfigError("Value out of config range")
        self.pulse = int(self._angle_to_pulse(angle))
        return self.pulse

    def get_angle(self, pulse = 0):
        """
        Get pulse angle. If pulse = 0 gets current motor pulse angle.
        :param pulse: in 1/4 micros
        :return: angle in radians
        """
        if pulse == 0:
            pulse = self.pulse
        pa = (self._calibration['min_pulse'] - self._calibration['max_pulse']) /\
             float(self._calibration['min_angle'] - self._calibration['max_angle'])
        return self._calibration['min_angle'] + ((pulse - self._calibration['min_pulse']) / pa)


    @staticmethod
    def get_default_pulse(angle):
        """
        :param angle: Angle in radians
        :return: pulse between 2800 - 9200 matching the -90 - 90 degrees angles
        """
        angle = max(min(math.pi / 2.0, angle), -math.pi / 2.0)
        return int(2800 + (9200 - 2800) * (angle + math.pi / 2.0) / math.pi)

    def get_calibrated_config(self):
        """
        :return: Array of config values with calibration data applied
        """
        conf= self._config
        conf['name'] = self.name
        conf['min'] = self. _calibration['min_angle']
        conf['max'] = self. _calibration['max_angle']
        conf['default'] = self.get_angle(self._config['init']*4)
        conf['motor_id'] = self.id
        return conf
