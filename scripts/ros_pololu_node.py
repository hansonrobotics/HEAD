#!/usr/bin/env python

__author__ = 'vytas'

import rospy
import math
import yaml
from pololu.motors import Maestro, MicroSSC
# Temporary messages
from ros_pololu_servo.msg import MotorCommand


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
        self._name = name
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
        self._pulse = int(self._config['init'] * 4)
        self._setup_calibration()
        self.id = self._config['motors_id']

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
        self._pulse = int(self._angle_to_pulse(angle))
        return self._pulse

    def get_angle(self):
        pa = (self._calibration['min_pulse'] - self._calibration['max_pulse']) /\
             (self._calibration['min_angle'] - self._calibration['max_angle'])
        return self._calibration['min_angle'] + ((self._pulse - self._calibration['min_pulse']) / pa)

    def get_pulse(self):
        return self._pulse

    @staticmethod
    def get_default_pulse(angle):
        """
        :param angle: Angle in radians
        :return: pulse between 3200 - 8800 matching the -90 - 90 degrees angles
        """
        angle = max(min(math.pi / 2.0, angle), -math.pi / 2.0)
        return int(3200 + (8800 - 3200) * (angle + math.pi / 2.0) / math.pi)

class RosPololuNode:

    def __init__(self):
        port = rospy.get_param("~port_name")
        topic_prefix = rospy.get_param("~topic_prefix", "pololu/")
        topic_name = rospy.get_param("~topic_name", "command")
        self._controller_type = rospy.get_param("~controller", "Maestro")
        self._motors = {}
        if rospy.has_param("~pololu_motors_yaml"):
            config_yaml = rospy.get_param("~pololu_motors_yaml")
            try:
                yaml_stream = open(config_yaml)
                config = yaml.load(yaml_stream)
                for name, config in config:
                    self._motors[name] = PololuMotor(name, config)
            except:
                rospy.logwarn("Error loading config files")
        try:
            if self._controller_type == 'Maestro':
                self.controller = Maestro(port)
            if self._controller_type == 'MicroSSC':
                self.controller = MicroSSC(port)
        except:
            rospy.logerr("Error creating the motor controller")
            return
        rospy.Subscriber(topic_prefix + topic_name, MotorCommand, self.motor_command_callback)

    def publish_motor_states(self):
        pass

    def motor_command_callback(self, msg):
        pulse = 0
        motor_id = 0
        if msg.joint_name in self._motors.keys():
            motor = self._motors[msg.joint_name]
            motor_id = motor.id
            pulse = motor.setAngle(msg.position)
        elif msg.joint_name.isdigit():
            try:
                motor_id = int(msg.joint_name)
            except:
                rospy.logwarn("Invalid motor specified")
            pulse = PololuMotor.get_default_pulse(msg.position)

        self.set_speed(motor_id, min(max(0, msg.speed), 1))
        self.set_acceleration(motor_id, min(max(0, msg.acceleration), 1))
        self.set_pulse(motor_id, pulse)


    def set_pulse(self, id, pulse):
        try:
            self.controller.setTarget(id, pulse)
        except AttributeError:
            pass


    def set_speed(self, id, speed):
        """
        Converting the speed 0-1 range to actual motor speed. Full speed would mean the motor would move full range in approx 0.1s. MicroSCC only allows twice slower speeds to be set.
        0 Speed sets pulse immediately.
        :param id: motor_id
        :param speed: speed
        """
        speed = int(512 * speed)
        try:
            self.controller.setSpeed(id, speed)
        except AttributeError:
            pass

    def set_acceleration(self, id, acceleation):

        acceleation = int(255 * acceleation)
        try:
            self.controller.setAcceleration(id, acceleation)
        except AttributeError:
            pass

if __name__ == '__main__':
    rospy.init_node("pololu_node")
    r = rospy.Rate(20)
    node = RosPololuNode()
    while not rospy.is_shutdown():
        node.publish_motor_states()
        r.sleep()
