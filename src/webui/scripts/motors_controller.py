#!/usr/bin/env python

import rospy
import os
import logging
import yaml
import json
from motors.configs import *
import webui.srv as srv
from rospkg import RosPack
from subprocess import Popen

rp = RosPack()

logger = logging.getLogger('hr.webui.bug_controller')
config_root = rp.get_path('robots_config')


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
        rospy.spin()

    def update_motors(self, req):
        configs = Configs()
        robot_name = req.robot_name
        configs.parseMotors(json.loads(req.motors))
        if len(configs.dynamixel) > 0:
            file_name = os.path.join(config_root, "dynamixel.yaml")
            write_yaml(file_name, configs.dynamixel)
            load_params(file_name, "/{}/safe".format(robot_name))
        if len(configs.motors) > 0:
            file_name = os.path.join(config_root, "motors.yaml")
            write_yaml(file_name, {'motors': configs.motors})
            load_params(file_name, "/{}".format(robot_name))
        if len(configs.pololu) > 0:
            for board, config in configs.pololu.iteritems():
                print config
                print board
                file_name = os.path.join(config_root, board + ".yaml")
                write_yaml(file_name, config)
                kill_node("/{}/pololu_{}".format(robot_name, board))
        kill_node("/{}/pau2motors".format(robot_name))

        return srv.UpdateMotorsResponse(True)


if __name__ == '__main__':
    MotorsController()
