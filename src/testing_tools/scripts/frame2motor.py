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

import MapperFactory, ParserFactory

df = pd.read_csv('shkey_frame_data.csv', header=None)
header=df.ix[:, 0]
df = df.transpose()
df = df.ix[1:]
df.columns = header

head_yaml = '{}/head.yaml'.format(os.path.join(HR_WORKSPACE, 'public_ws/src/robots_config/sophia'))

with open(head_yaml) as stream:
    motor_configs = yaml.load(stream)


shkey_motors = []
for motor in sorted(motor_configs.keys()):
    motor_entry = motor_configs[motor]
    parser_cfg = motor_entry['pau']['parser']
    parser_name = parser_cfg['name']
    if parser_name == 'fsshapekey':
        shkey_motors.append(motor)

df2 = pd.DataFrame(columns=shkey_motors)
for i in range(len(df)):
    frame = df.ix[i+1]
    mapped_values = []
    for motor in shkey_motors:
        motor_entry = motor_configs[motor]
        parser_cfg = motor_entry['pau']['parser']
        shkeys = parser_cfg['shapekey'].split(';')
        values = [frame[key] for key in shkeys]
        if len(values) == 1:
            values = values[0]
        mapper = MapperFactory.build(motor_entry['pau']['function'], motor_entry)
        mapped_value = mapper.map(values)
        mapped_values.append(mapped_value)
        print '%s %s -> %s' % (motor, values, mapped_value)

    df2.loc[len(df2)] = mapped_values

df2.to_csv('shkey_motor_data.csv', index=False)

