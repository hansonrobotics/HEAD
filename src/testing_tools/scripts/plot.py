#!/usr/bin/env python
#
# Plot curves
#
import pandas as pd
import matplotlib.pyplot as plt
import os
from frame2motor import frame2motor, load_motor_configs, get_shkey_motors

HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
CWD = os.path.abspath(os.path.dirname(__file__))

motor_configs = load_motor_configs()
shkey_motors = get_shkey_motors(motor_configs)

def plot(shkey_data_file, pau_motor_file, serial_port_data_file, output_dir):
    """serial commands vs. shape keys"""
    id2motor = {motor_configs[name]['motor_id']: name for name in shkey_motors}

    df = pd.read_csv(serial_port_data_file)
    groups = df[df.Command == 'position'].groupby('MotorID')
    msgs_group = groups.groups
    shkey_df = pd.read_csv(shkey_data_file)
    pau_df = pd.read_csv(pau_motor_file)

    line_prop = {'linewidth': 1, 'marker': 'o', 'markersize': 2}
    for motor_id, rows in msgs_group.items():
        if motor_id not in id2motor: continue
        motor = id2motor[motor_id]
        f, axs = plt.subplots(2, figsize=(14, 10))
        f.suptitle("Motor %s, ID %s" % (motor, motor_id), fontsize=14)
        y = df.ix[rows].Value.tolist()
        pau = (pau_df[motor]*4).tolist()
        axs[0].plot(y, label='Serial Port', **line_prop)
        axs[0].plot(pau, label='Pau Message', **line_prop)
        axs[0].yaxis.grid()
        axs[0].legend()

        shkey = (shkey_df[motor]*4).tolist()
        axs[1].plot(shkey, label='Shape Key', **line_prop)
        axs[1].yaxis.grid()
        axs[1].legend()

        plt.tight_layout()
        plt.subplots_adjust(top=0.95)
        f.savefig(os.path.join(output_dir, '%s.png' % motor), dpi=80)

if __name__ == '__main__':
    shkey_data_file = os.path.join(CWD, 'shkey_motor_data.csv')
    serial_port_data_file = os.path.join(CWD, 'serial_port_data.csv')
    pau_data_file = os.path.join(CWD, 'pau_data.csv')
    pau_motor_file = os.path.join(CWD, 'pau_motor.csv')
    frame2motor(pau_data_file, pau_motor_file)
    output_dir = '%s/fig' % CWD
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    plot(shkey_data_file, pau_motor_file, serial_port_data_file, output_dir)

