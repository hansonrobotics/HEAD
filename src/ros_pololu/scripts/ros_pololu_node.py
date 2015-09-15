#!/usr/bin/env python

__author__ = 'vytas'

import rospy
import math
import yaml
from pololu.motors import Maestro, MicroSSC
# Temporary messages
from ros_pololu.msg import MotorCommand
import time

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
        :return: pulse between 3200 - 8800 matching the -90 - 90 degrees angles
        """
        angle = max(min(math.pi / 2.0, angle), -math.pi / 2.0)
        return int(3200 + (8800 - 3200) * (angle + math.pi / 2.0) / math.pi)

    def get_calibrated_config(self):
        """
        :return: Array of config values with calibration data applied
        """
        conf= self._config
        conf['name'] = self._name
        conf['min'] = self. _calibration['min_angle']
        conf['max'] = self. _calibration['max_angle']
        conf['default'] = self.get_angle(self._config['init']*4)
        conf['motor_id'] = self.id
        return conf



class RosPololuNode:

    def __init__(self):
        port = rospy.get_param("~port_name")
        topic_prefix = rospy.get_param("~topic_prefix", "pololu/")
        topic_name = rospy.get_param("~topic_name", "command")
        # Use safety proxy to filter motor comamnds
        safety = rospy.get_param("~safety", False)
        # Use specific rate to publish motors commands
        self._sync = rospy.get_param("~sync", "off")
        self._controller_type = rospy.get_param("~controller", "Maestro")
        self._motors = {}
        self.idle = False
        if rospy.has_param("~pololu_motors_yaml"):
            config_yaml = rospy.get_param("~pololu_motors_yaml")
            try:
                yaml_stream = open(config_yaml)
                config = yaml.load(yaml_stream)
            except:
                rospy.logwarn("Error loading config files")
            # Get existing motors config and update those configs if callibration data changed
            motors = rospy.get_param('motors',[])
            for name, cfg in config.items():
                self._motors[name] = PololuMotor(name, cfg)
                cfg = self._motors[name].get_calibrated_config()
                cfg['topic'] = topic_prefix.strip("/")
                cfg['hardware'] = 'pololu'
                for i, m in enumerate(motors):
                    if m['name'] == name:
                        motors[i] = cfg
                        break
                else:
                    motors.append(cfg)
            print(motors)
            rospy.set_param('motors', motors)
        try:
            if self._controller_type == 'Maestro':
                self.controller = Maestro(port)
                print "Maestro started"
            if self._controller_type == 'MicroSSC':
                self.controller = MicroSSC(port)
                print "MicroSSC started"
        except Exception as ex:
            rospy.logwarn("Error creating the motor controller")
            rospy.logwarn(ex)
            self.idle = True
            return
        # Listen for outputs from proxy
        if safety:
            topic_prefix = 'safe/'+topic_prefix
        rospy.Subscriber(topic_prefix + topic_name, MotorCommand, self.motor_command_callback)
        rospy.loginfo("ros_pololu Subscribed to %s" % (topic_prefix + topic_name))

    def publish_motor_states(self):
        if self._sync == 'on':
            for i, m in self._motors.items():
                try:
                    self.set_speed(m.id, m.speed)
                    self.set_acceleration(m.id, m.acceleration)
                    self.set_pulse(m.id, m.pulse)
                except:
                    rospy.logerr("Write Timeout")
                    time.sleep(0.01)
                    self.controller.clean()


            self.controller.clean()

    def motor_command_callback(self, msg):
        if self.idle:
            return
        pulse = 0
        motor_id = 0
        if msg.joint_name in self._motors.keys():
            motor = self._motors[msg.joint_name]
            motor_id = motor.id
            pulse = motor.set_angle(msg.position)
            #motor.speed =  min(max(0, msg.speed), 1)
            motor.acceleration = min(max(0, msg.acceleration), 1)
            if msg.speed > 1:
                msg.speed = motor.speed
            if msg.acceleration > 1:
                msg.acceleration = motor.acceleration
        elif msg.joint_name.isdigit():
            try:
                motor_id = int(msg.joint_name)
            except:
                rospy.logwarn("Invalid motor specified")
                return
            pulse = PololuMotor.get_default_pulse(msg.position)

        if not (self._sync == 'on' and (msg.joint_name in self._motors.keys())):
            self.set_speed(motor_id, min(max(0, msg.speed), 1))
            self.set_acceleration(motor_id, min(max(0, msg.acceleration), 1))
            self.set_pulse(motor_id, pulse)



    def set_pulse(self, id, pulse):
        try:
            print id
            print pulse
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

        acceleation = 0 # FIXME: disable acceleration because pololu may have problem with acceleration
        try:
            self.controller.setAcceleration(id, acceleation)
        except AttributeError:
            pass

if __name__ == '__main__':
    rospy.init_node("pololu_node")
    r = rospy.Rate(30)
    # Adding delay in order to avoid nodes loading at same time
    delay = rospy.get_param('~delay', 0)
    time.sleep(delay)
    node = RosPololuNode()
    while not rospy.is_shutdown():
        node.publish_motor_states()
        r.sleep()
