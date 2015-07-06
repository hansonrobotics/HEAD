#!/usr/bin/env python

import rospy
from ros_pololu.msg import MotorCommand
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList

import time



class Safety():

    ros_rate = 20

    def __init__(self):
        self.topics = []
        # Wait for motors to be loaded in param server
        time.sleep(3)
        motors = rospy.get_param('motors')
        self.rules = rospy.get_param('safety_rules', {})
        # subscribe
        self.motor_positions = {}
        self.subscribers = {}
        self.publishers = {}
        self.motors = {}
        self.timers = {}
        self.motors_msgs = {}
        # Dynamixel loads
        self.motor_loads = [0]*256
        # Create proxy topics and subscribers
        for m in motors:
            self.motor_positions[m['name']] = m['default']
            self.motors[m['name']] = m
            if not m['topic'] in self.publishers.keys():
                # Pololu motor if motor_id is specified
                if 'motor_id' in m:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"/command",MotorCommand, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"/command", MotorCommand,
                                    lambda msg,m=m: self.callback(m, False, msg))
                else:
                    self.publishers[m['topic']] = rospy.Publisher("safe/"+m['topic']+"_controller/command",Float64, queue_size=30)
                    self.subscribers[m['topic']] = rospy.Subscriber(m['topic']+"_controller/command", Float64,
                                        lambda msg, m=m: self.callback(m, True, msg))
        # Subscribe motor states
        rospy.Subscriber('safe/motor_states', MotorStateList, self.update_load)
        # Init timing rules
        for m, rules in self.rules.iteritems():
            for i,r in enumerate(rules):
                # Process timing rules
                if r['type'] == 'timing':
                    # Init rule variables
                    self.rules[m][i]['started'] = False
                    self.rules[m][i]['limit'] = 1
                if r['type'] == 'load':
                    self.rules[m][i]['started'] = False
                    self.rules[m][i]['limit'] = 1

    def update_load(self, msg):
        for s in msg.motor_states:
            self.motor_loads[s['id']] = s['load']

    # check if motor is dynamixel
    def is_dynamixel(self, m):
        if 'motor_id' in self.motors[m]:
            return False
        else:
            return True

    def callback(self, motor, dynamixel, msg):
        # Republish message to safe topic
        mname = motor['name']
        if mname in self.rules.keys():
            self.process_rules(mname, dynamixel, msg)
        if dynamixel:
            v = msg.data
        else:
            v = msg.position
        self.motor_positions[mname] = v
        self.motors_msgs[mname] = msg
        self.publishers[motor['topic']].publish(msg)

    def process_rules(self, motor, dynamixel, msg):
        if dynamixel:
            v = msg.data
        else:
            v = msg.position
        rules = self.rules[motor]
        for r in rules:
            if r['type'] == 'prevent':
                v = self.rule_prevent(motor, v, r)
            if (r['type'] == 'timing') or (r['type'] == 'load'):
                v = self.rule_time(motor, v, r)
        if dynamixel:
            msg.data = v
        else:
            msg.position = v
    #Prevents motor going over its safe range if the other oposite motors are set to different directions
    def rule_prevent(self, motor, v, rule):
        # Check if its over extreme
        if self.get_relative_pos(motor, rule['direction'], v) <= rule['extreme']:
            return v
        # Check dependency
        if self.get_relative_pos(rule['depends'], rule['dep_dir'], self.motor_positions[rule['depends']]) > rule['dep_extreme']:
            # Preventing form getting over limit. sets motor position to limit before extreme
            return self.get_abs_pos(motor, rule['direction'], rule['extreme'])
        return v

    # Check if motor is not over current limit set by existing rule. Applies for timing and load rules
    def rule_time(self, motor, v, rule):
        # Check if its over limit
        relative = self.get_relative_pos(motor, rule['direction'], v)

        if relative > rule['limit']:
            return self.get_abs_pos(motor, rule['direction'], rule['limit'])
        return v

    # Gets absolute position from relative between the neutral and extreme in given direction
    def get_abs_pos(self, motor, direction, v):
        extreme = self.motors[motor][direction]
        return self.motors[motor]['default'] + (extreme - self.motors[motor]['default']) * v
    # Gets relative position from abs to the direction of given extreme

    def get_relative_pos(self, motor, direction, v):
        extreme = self.motors[motor][direction]
        # avoid div by 0
        if extreme == self.motors[motor]['default']:
            return 0
        return (v - self.motors[motor]['default']) / (extreme - self.motors[motor]['default'])

    # Scheduled tasks
    def timing(self):
        for m, rules in self.rules.iteritems():
            for i,r in enumerate(rules):
                # Process timing rules
                if r['type'] == 'timing':
                    self.rule_timing(m,i)
                if r['type'] == 'load':
                    self.rule_loading(m,i)

    def rule_timing(self, m, r):
        # Rule is active
        rule = self.rules[m][r]
        if rule['started']:
            # Servo still in extreme
            relative = self.get_relative_pos(m, rule['direction'], self.motor_positions[m])
            extreme =  relative > rule['extreme']
            if rule['started'] + rule['t1'] > time.time():
                if extreme:
                    return
                else:
                    # if moved out of extreme cancel timer
                    self.rules[m][r]['started'] = False
                    self.rules[m][r]['limit'] = 1
                    return
            # Rule expired
            if rule['started'] + rule['t1']+rule['t2']+rule['t3']+rule['t4'] < time.time():
                 self.rules[m][r]['started'] = False
                 self.rules[m][r]['limit'] = 1
                 return
            limit = rule['extreme']
            # Limit decreased for safe position
            if rule['started'] + rule['t1']+rule['t2'] > time.time():
                limit = limit + (1-limit)*(rule['started'] + rule['t1']  + rule['t2'] - time.time())/rule['t2']
            # Period increased for extreme position to be available
            if rule['started'] + rule['t1']+rule['t2']+rule['t3'] < time.time():
                limit = limit + (1-limit)*(time.time()- rule['started'] - rule['t1']  - rule['t2'] - rule['t3'])/rule['t4']
            self.rules[m][r]['limit'] = limit
            # Set position if limit is lower than current
            if limit < relative:
                self.set_motor_relative_pos(m, limit, rule['direction'])
        else:
            #Over extreme position, start the tracking
            if self.get_relative_pos(m, self.rules[m][r]['direction'], self.motor_positions[m]) > self.rules[m][r]['extreme']:
                self.rules[m][r]['started'] = time.time()

    def rule_loading(self,m,r):
        rule = self.rules[m][r]
        dir = 1 if rule['direction']=='max' else -1
        extreme = self.motor_loads[rule['motor_id']]*dir > rule['extreme']
        if rule['started']:
            # Allow t1 time for rule to take meassures
            if rule['started'] + rule['t1'] > time.time():
                if extreme:
                    return
                else:
                    self.rules[m][r]['started'] = False
                    self.rules[m][r]['limit'] = 1
                    return
            limit = 1.0
            if rule['started'] + rule['t1']+rule['t2'] > time.time():
                if rule['limit'] == 1:
                    # Prevent motor for going further
                    limit = self.get_relative_pos(m,rule['direction'], self.motor_positions['m'])
                if extreme:
                    # Rapidly decrease limit towards neutral
                    limit -= 1.0 / self.ros_rate
                limit = max(0,limit)
            else:
                # Increase limit gradually
                limit += 1.0/ self.ros_rate
                if limit >= 1:
                    #Rule Expired
                    limit = 1
                    self.rules[m][r]['started'] = False

            relative = self.get_relative_pos(m, rule['direction'], self.motor_positions[m])
            self.rules[m][r]['limit'] = limit
            if relative > limit:
                self.set_motor_relative_pos(m,relative,rule['direction'])
        else:
            if extreme:
                self.rules[m][r]['started'] = time.time()
        pass

    def set_motor_relative_pos(self, m, pos, dir):
        v = self.get_abs_pos(m,dir,pos)
        self.set_motor_abs_pos(m,v)

    def set_motor_abs_pos(self, m, pos):
        msg = self.motors_msgs[m]
        if self.is_dynamixel(m):
            msg.data = pos
        else:
            msg.position = pos
        self.motor_positions[m]= pos
        self.publishers[self.motors[m]['topic']].publish(msg)


