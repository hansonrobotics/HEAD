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

def plot(shkey_data_file, pau_motor_file, serial_port_data_file, output_dir):
    """serial commands vs. shape keys"""
    serial_df = pd.read_csv(serial_port_data_file)
    shkey_df = pd.read_csv(shkey_data_file)
    pau_df = pd.read_csv(pau_motor_file)

    def get_yrange(y):
        yrange = (max(y)-min(y))
        ymin, ymax = min(y)-0.1*yrange, max(y)+0.1*yrange
        return ymin, ymax

    line_prop = {'linewidth': 2, 'markersize': 4, 'alpha': 0.6}
    for motor, rows in serial_df.iteritems():
        f, axs = plt.subplots(2, figsize=(14, 10))
        f.suptitle("Motor %s" % motor, fontsize=14)
        y = rows.tolist()
        pau = (pau_df[motor]*4).astype(int).tolist()
        axs[0].plot(y, label='Serial Port', marker='o', **line_prop)
        axs[0].plot(pau, label='Pau Message', marker='*', **line_prop)
        axs[0].yaxis.grid()
        ymin, ymax = get_yrange(y)
        ymin2, ymax2 = get_yrange(pau)
        axs[0].set_ylim([min(ymin, ymin2), max(ymax, ymax2)])
        axs[0].legend()

        shkey = (shkey_df[motor]*4).astype(int).tolist()
        axs[1].plot(shkey, label='Shape Key', marker='o', **line_prop)
        axs[1].yaxis.grid()
        ymin, ymax = get_yrange(shkey)
        axs[1].set_ylim([ymin, ymax])
        axs[1].legend()

        plt.tight_layout()
        plt.subplots_adjust(top=0.95)
        f.savefig(os.path.join(output_dir, '%s.png' % motor), dpi=80)

if __name__ == '__main__':
    shkey_data_file = os.path.join(CWD, 'shkey_frame_motor_data.csv')
    serial_port_data_file = os.path.join(CWD, 'serial_port_data.csv')
    pau_data_file = os.path.join(CWD, 'pau_data.csv')
    pau_motor_file = os.path.join(CWD, 'pau_motor.csv')
    frame2motor(pau_data_file, pau_motor_file)
    output_dir = '%s/fig' % CWD
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    plot(shkey_data_file, pau_motor_file, serial_port_data_file, output_dir)

