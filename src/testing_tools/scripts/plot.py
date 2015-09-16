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

    shkey_df = pd.read_csv(os.path.join(CWD, 'shkey_motor_data.csv'))
    pau_df = pd.read_csv(os.path.join(CWD, 'pau_data.csv'))
    line_prop = {'linewidth': 1, 'marker': 'o', 'markersize': 2}
    for motor_id, rows in msgs_group.items():
        if motor_id not in id2motor: continue
        motor = id2motor[motor_id]
        f, axs = plt.subplots(3, figsize=(20, 16))
        f.suptitle("Motor %s" % motor, fontsize=14)
        y = df.ix[rows].Value.tolist()
        axs[0].plot(y, **line_prop)
        axs[0].set_title("Motor command")
        axs[0].yaxis.grid()

        pau = (pau_df[motor]*4).tolist()
        axs[1].plot(pau, **line_prop)
        axs[1].set_title("Pau Message")
        axs[1].yaxis.grid()

        shkey = (shkey_df[motor]*4).tolist()
        axs[2].plot(shkey, **line_prop)
        axs[2].set_title("Shape key")
        axs[2].yaxis.grid()

        plt.tight_layout()
        plt.subplots_adjust(top=0.95)
        f.savefig(os.path.join(output_dir, '%s.png' % motor), dpi=80)

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
        f, axs = plt.subplots(2, figsize=(12, 16))
        f.suptitle("Motor %s" % motor, fontsize=14)
        y = [msg.position for msg in msgs]
        axs[0].plot(y)
        axs[0].set_title("Motor command")

        y2 = df[motor].tolist()
        axs[1].plot(y2)
        axs[1].set_title("Shape key")
        f.savefig(os.path.join(output_dir, '%s.png' % motor), dpi=100)

if __name__ == '__main__':
    plot()

