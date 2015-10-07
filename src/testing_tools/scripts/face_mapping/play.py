import os
import numpy as np
import pandas as pd
from fit import get_motors, init_position, set_servos
import time
from pololu.motors import Maestro

def set_position(motors, pos_series):
    active_motors = pos_series.index.tolist()
    for motor in motors:
        if motor['name'] in active_motors:
            pos = pos_series[motor['name']]
            if not np.isnan(pos):
                motor['pos'] = pos
                print "set {} to {}".format(motor['name'], pos)

if __name__ == '__main__':
    signal_file = 'signal.csv'
    signal_df = pd.read_csv(signal_file)
    motors = get_motors('head.yaml')
    init_position(motors)

    DEVICE = '/dev/ttyACM0'
    controller = None
    if os.path.exists(DEVICE):
        controller = Maestro(DEVICE)

    for index, row in signal_df.iterrows():
        set_position(motors, row)
        if controller:
            set_servos(controller, motors)
        time.sleep(0.5)
