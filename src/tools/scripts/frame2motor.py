#!/usr/bin/env python
#
# Convert face frame data to motor frame data
#

import pandas as pd
import yaml
import os
import sys
HR_WORKSPACE = os.environ.get('HR_WORKSPACE', os.path.expanduser('~/hansonrobotics'))
sys.path.insert(0, os.path.join(HR_WORKSPACE, 'public_ws/src/pau2motors/src'))
import MapperFactory

def load_motor_configs():
    head_yaml = '{}/head.yaml'.format(
        os.path.join(HR_WORKSPACE, 'public_ws/src/robots_config/sophia'))
    with open(head_yaml) as stream:
        motor_configs = yaml.load(stream)
    return motor_configs

def get_shkey_motors(motor_configs):
    shkey_motors = []
    for motor in sorted(motor_configs.keys()):
        motor_entry = motor_configs[motor]
        parser_cfg = motor_entry['pau']['parser']
        parser_name = parser_cfg['name']
        if parser_name == 'fsshapekey':
            shkey_motors.append(motor)
    return shkey_motors

def frame2motor(ifile, ofile):
    motor_configs = load_motor_configs()
    shkey_motors = get_shkey_motors(motor_configs)

    def shkey_name_mapping(shkeys):
        """Map motor shkeys to blender shkeys"""
        mapping = {'brow_outer_UP.R': 'brow_outer_up.R'}
        bl_shkeys = [mapping[key] if key in mapping else key for key in shkeys]
        return bl_shkeys

    df = pd.read_csv(ifile)
    df2 = pd.DataFrame(columns=shkey_motors)
    for i in range(len(df)):
        frame = df.ix[i]
        mapped_values = []
        for motor in shkey_motors:
            motor_entry = motor_configs[motor]
            parser_cfg = motor_entry['pau']['parser']
            shkeys = parser_cfg['shapekey'].split(';')
            values = [frame[key] for key in shkey_name_mapping(shkeys)]
            if len(values) == 1:
                values = values[0]
            mapper = MapperFactory.build(motor_entry['pau']['function'], motor_entry)
            mapped_value = mapper.map(values)
            mapped_values.append(mapped_value)
            print '%s %s -> %s' % (motor, values, mapped_value)
        df2.loc[len(df2)] = mapped_values

    df2.to_csv(ofile, index=False)
    print "Write to {}".format(ofile)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print >> sys.stderr, "Not enough arguments"
        print >> sys.stderr, "Usage: ./{} ifile ofile".format(sys.argv[0])
        sys.exit(1)
    ifile = sys.argv[1]
    ofile = sys.argv[2]

    frame2motor(ifile, ofile)

