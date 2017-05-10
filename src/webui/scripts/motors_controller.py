#!/usr/bin/env python

import rospy
import os
import logging
import yaml
import json
from motors.configs import *
import webui.srv as srv
from subprocess import Popen

logger = logging.getLogger('hr.webui.bug_controller')

def write_yaml(filename, data):
    # delete existing config
    try:
        os.remove(filename)
    except OSError:
        pass

    dir = os.path.dirname(filename)
    if not os.path.exists(dir):
        os.makedirs(dir)

    f = open(filename, 'w')
    f.write(yaml.safe_dump(data, encoding='utf-8', allow_unicode=True))
    f.close()

def load_params(param_file, namespace):
    Popen("rosparam load " + param_file + " " + namespace, shell=True)


def kill_node(node):
    Popen("rosnode kill " + node, shell=True)


class MotorsController:
    def __init__(self):
        rospy.init_node('motors_controller')
        rospy.Service('~update_motors', srv.UpdateMotors, self.update_motors)
        rospy.Service('~update_expressions', srv.UpdateExpressions, self.update_expressions)
        self.config_root = rospy.get_param('/robots_config_dir')
        rospy.spin()

    def update_motors(self, req):
        configs = Configs()
        robot_name = req.robot_name
        configs.parseMotors(json.loads(req.motors))
        if len(configs.dynamixel) > 0:
            file_name = os.path.join(self.config_root, robot_name, "dynamixel.yaml")
            write_yaml(file_name, configs.dynamixel)
            load_params(file_name, "/{}/safe".format(robot_name))
        if len(configs.motors) > 0:
            file_name = os.path.join(self.config_root, robot_name, "motors.yaml")
            write_yaml(file_name, {'motors': configs.motors})
            load_params(file_name, "/{}".format(robot_name))
        if len(configs.pololu) > 0:
            for board, config in configs.pololu.iteritems():
                file_name = os.path.join(self.config_root, robot_name, board + ".yaml")
                write_yaml(file_name, config)
                kill_node("/{}/pololu_{}".format(robot_name, board))
        kill_node("/{}/pau2motors".format(robot_name))
        kill_node("/{}/basic_head_api".format(robot_name))
        return srv.UpdateMotorsResponse(True)

    def update_expressions(self, req):
        robot_name = req.robot_name
        expressions = os.path.join(self.config_root, robot_name, "expressions.yaml")
        load_params(expressions, "/{}".format(robot_name))
        animations = os.path.join(self.config_root, robot_name, "animations.yaml")
        load_params(animations, "/{}".format(robot_name))
        kill_node("/{}/basic_head_api".format(robot_name))
        return srv.UpdateExpressionsResponse(True)


if __name__ == '__main__':
    MotorsController()
