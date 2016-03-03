#!/usr/bin/env python

import rospy
import logging
import time
import math
from dynamixel_controllers.srv import TorqueEnable, TorqueEnableResponse
from dynamixel_msgs.msg import MotorStateList
from webui.srv import *
logger = logging.getLogger('hr.webui.node_configuration')
dynlog = logging.getLogger('hr.dynamixel_data')
dynlog.setLevel(logging.ERR)

class DxlController:

    def __init__(self):
        logger.info('Starting dynamixel interface node')
        self.motors = rospy.get_param('motors')
        rospy.init_node('node_configuration')
        rospy.Service('set_dxl_torque', TorqueEnable, self.set_dxl_torque)
        self.last_state = []
        self.last_state_time = 0
        rospy.Subscriber('safe/motor_states/default', MotorStateList, self.update_motor_states)
        rospy.Service('get_motor_states', MotorStates, self.get_motor_states)
        rospy.spin()


    # Forwards service to all dynamixel services
    def set_dxl_torque(self, msg):
        for m in self.motors:
            if m['hardware'] == 'dynamixel':
                service = 'safe/{}_controller/torque_enable'.format(m['name'])
                try:
                    srv = rospy.ServiceProxy(service, TorqueEnable)
                    srv(msg.torque_enable)
                except:
                    logger.warn("No service available")
                    print service
        return TorqueEnableResponse()

    # Saves last available motor states
    def update_motor_states(self, req):
        self.last_state = req.motor_states
        self.last_state_time = time.time()
        self.logStates(req.motor_states)

    def logStates(self, states):
        for state in states:
            dynlog.info(",".join([str(state.id), str(state.goal), str(state.error), \
                                  str(state.load), str(state.voltage), str(state.temperature)]))

    def get_motor_states(self, req):
        states = MotorStatesResponse()
        # If state is very recent only
        if self.last_state_time > time.time() - 0.5:
            state_list = self.last_state
            #for state in state_list
            for state in state_list:
                name, angle = self.get_motor_name_angle(state)
                states.motors.append(name)
                states.angles.append(angle)

        return states

    #
    def get_motor_name_angle(self, state):
        for m in self.motors:
            if (m['hardware'] == 'dynamixel') & (m['motor_id'] == state.id):
                angle = self.dynamixel_angle(m, state.position)
                name = m['name']
                return name, angle

    @staticmethod
    def dynamixel_angle(m, v):
        return (v - m['init']) * math.pi / 2048

if __name__ == '__main__':
    DxlController()




