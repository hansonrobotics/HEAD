#!/usr/bin/env python
#
# Plot curves
#
import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import sys
import os
from collections import defaultdict
import yaml

HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
CWD = os.path.abspath(os.path.dirname(__file__))

head_yaml = '{}/head.yaml'.format(os.path.join(HR_WORKSPACE, 'public_ws/src/robots_config/sophia'))
output_dir = '%s/fig' % CWD

with open(head_yaml) as stream:
    motor_configs = yaml.load(stream)

shkey_motors = []
for motor in sorted(motor_configs.keys()):
    motor_entry = motor_configs[motor]
    parser_cfg = motor_entry['pau']['parser']
    parser_name = parser_cfg['name']
    if parser_name == 'fsshapekey':
        shkey_motors.append(motor)

if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

def plot():
    """serial commands vs. shape keys"""
    id2motor = {motor_configs[name]['motor_id']: name for name in shkey_motors}
    df = pd.read_csv(os.path.join(CWD, 'yawn-1-serial-commands.csv'))
    groups = df[df.Command == 'position'].groupby('MotorID')
    msgs_group = groups.groups

    df2 = pd.read_csv(os.path.join(CWD, 'shkey_motor_data.csv'))
    for motor_id, rows in msgs_group.items():
        if motor_id not in id2motor: continue
        motor = id2motor[motor_id]
        f, axs = plt.subplots(2)
        f.suptitle("Motor %s" % motor, fontsize=14)
        y = df.ix[rows].Value.tolist()
        axs[0].plot(y)
        axs[0].set_title("Motor command")
        axs[0].yaxis.grid()

        y2 = (df2[motor]*4).tolist()
        axs[1].plot(y2)
        axs[1].set_title("Shape key")
        axs[1].yaxis.grid()
        f.savefig(os.path.join(output_dir, '%s.png' % motor))

def plot2():
    """/sophia/safe/head/command vs. shape keys"""
    bag_file = os.path.join(CWD, 'yawn-1_head_command.bag')
    bag = rosbag.Bag(bag_file)
    msgs_group = defaultdict(list)
    for topic, message, timestamp in bag.read_messages():
        if message.joint_name in shkey_motors:
            msgs_group[message.joint_name].append(message)

    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    df = pd.read_csv(os.path.join(CWD, 'shkey_motor_data.csv'))
    for motor, msgs in msgs_group.iteritems():
        f, axs = plt.subplots(2)
        f.suptitle("Motor %s" % motor, fontsize=14)
        y = [msg.position for msg in msgs]
        axs[0].plot(y)
        axs[0].set_title("Motor command")

        y2 = df[motor].tolist()
        axs[1].plot(y2)
        axs[1].set_title("Shape key")
        f.savefig(os.path.join(output_dir, '%s.png' % motor))

if __name__ == '__main__':
    plot()

