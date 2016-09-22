#!/usr/bin/env python

__author__ = 'vytas'

import rospy
import math
import yaml
from pololu.motors import Maestro, MicroSSC
from ros_pololu.msg import MotorCommand
from sensor_msgs.msg import JointState
from ros_pololu import PololuMotor
import time
import logging

COMMAND_RATE = 24
logger = logging.getLogger('hr.ros_pololu.ros_pololu_node')

class RosPololuNode:

    def __init__(self):
        port = rospy.get_param("~port_name")
        topic_prefix = rospy.get_param("~topic_prefix", "pololu/")
        topic_name = rospy.get_param("~topic_name", "command")
        # Use safety proxy to filter motor comamnds
        safety = rospy.get_param("~safety", False)
        # Use specific rate to publish motors commands
        self._sync = rospy.get_param("~sync", "off")
        self._dynamic_speed = rospy.get_param("~dyn_speed", "off")
        self._servo_rate = rospy.get_param("~servo_rate", 50)
        self._controller_type = rospy.get_param("~controller", "Maestro")
        self._hw_fail = rospy.get_param("~hw_required", False)

        self._motors = {}
        self.idle = False
        if rospy.has_param("~pololu_motors_yaml"):
            config_yaml = rospy.get_param("~pololu_motors_yaml")
            try:
                yaml_stream = open(config_yaml)
                config = yaml.load(yaml_stream)
            except:
                logger.warn("Error loading config files")
            # Get existing motors config and update those configs if callibration data changed
            motors = rospy.get_param('motors',[])

            for name, cfg in config.items():
                self._motors[name] = PololuMotor(name, cfg)
            #     cfg = self._motors[name].get_calibrated_config()
            #     cfg['topic'] = topic_prefix.strip("/")
            #     cfg['hardware'] = 'pololu'
            #     for i, m in enumerate(motors):
            #         if m['name'] == name:
            #             motors[i] = cfg
            #             break
            #     else:
            #         motors.append(cfg)
            # rospy.set_param('motors', motors)
        try:
            if self._controller_type == 'Maestro':
                self.controller = Maestro(port)
            if self._controller_type == 'MicroSSC':
                self.controller = MicroSSC(port)
        except Exception as ex:
            if self._hw_fail:
                rospy.logfatal("Hardware needs to be attached")
                rospy.signal_shutdown("HW Failure")
                return
            logger.warn("Error creating the motor controller")
            logger.warn(ex)
            self.idle = True
            rospy.set_param(topic_prefix.strip("/")+"_enabled",False)
            return
        rospy.set_param(topic_prefix.strip("/")+"_enabled",True)
        # Listen for outputs from proxy
        self.states_pub = rospy.Publisher(topic_prefix + "motors_states", JointState, queue_size=10)
        if safety:
            topic_prefix = 'safe/'+topic_prefix
        rospy.Subscriber(topic_prefix + topic_name, MotorCommand, self.motor_command_callback)
        logger.info("ros_pololu Subscribed to %s" % (topic_prefix + topic_name))

    def publish_motor_states(self):
        if self.idle:
            return
        if self._sync == 'on':
            for i, m in self._motors.items():
                try:
                    if self._dynamic_speed == "on":
                        # Get speed required and normalize it
                        speed = Maestro.calculateSpeed(m.last_pulse, m.pulse, 1.0/COMMAND_RATE, 1.0/self._servo_rate) / 512.0
                        m.last_pulse = m.pulse
                    else:
                        speed = m.speed
                    self.set_speed(m.id, speed)
                    self.set_pulse(m.id, m.pulse)
                    self.set_acceleration(m.id, 0)

                except Exception as ex:
                    logger.error("Error %s" % ex)
                    time.sleep(0.01)
                    self.controller.clean()
            self.controller.clean()

        # Publish the states
        msg = JointState()
        for i, m in self._motors.items():
            msg.name.append(m.name)
            msg.position.append(m.get_angle())

        self.states_pub.publish(msg)


    def motor_command_callback(self, msg):
        # Enable command processing for debuging
        if self.idle:
            return
        pulse = 0
        motor_id = 0
        if msg.joint_name in self._motors.keys():
            motor = self._motors[msg.joint_name]
            motor_id = motor.id
            pulse = motor.set_angle(msg.position)
            motor.speed =  min(max(0, msg.speed), 1)
            motor.acceleration = min(max(0, msg.acceleration), 1)
            if msg.speed > 1:
                msg.speed = motor.speed
            if msg.acceleration > 1:
                msg.acceleration = motor.acceleration
        elif msg.joint_name.isdigit():
            try:
                motor_id = int(msg.joint_name)
            except:
                logger.warn("Invalid motor specified")
                return
            pulse = PololuMotor.get_default_pulse(msg.position)

        if not (self._sync == 'on' and (msg.joint_name in self._motors.keys())):
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
        # FIXME: disable acceleration because pololu may have problem with acceleration
        acceleration = 0
        try:
            self.controller.setAcceleration(id, acceleration)
        except AttributeError:
            pass

if __name__ == '__main__':
    rospy.init_node("pololu_node")
    COMMAND_RATE =rospy.get_param('~command_rate', COMMAND_RATE)
    r = rospy.Rate(COMMAND_RATE)
    # Adding delay in order to avoid nodes loading at same time
    delay = rospy.get_param('~delay', 0)
    time.sleep(delay)
    node = RosPololuNode()
    while not rospy.is_shutdown():
        node.publish_motor_states()
        r.sleep()
