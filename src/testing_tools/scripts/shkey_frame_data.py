#!/usr/bin/env python3
#
# This script should be run in blender.
# It fetches the face data of 200 frames for a specific animation.
#

import roscom
import bpy
from collections import defaultdict
import time

msgs = defaultdict(list)

for i in range(1, 201):
    bpy.context.scene.frame_set(i)
    time.sleep(0.01)
    data=roscom.api.getFaceData()
    print("frame", i)
    for k, v in data.items():
        msgs[k].append(v)
with open('shkey_frame_data.csv', 'w') as f:
    for k, v in msgs.items():
        f.write('{},{}\n'.format(k, ','.join(map(str, v))))
print('write done')

