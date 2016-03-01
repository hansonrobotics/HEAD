#!/usr/bin/env python3
import rospy
import yaml
from rospkg import RosPack
ROBOT = "sophia_body"
#import bpy
#rospy.init_node("expression_tools")
motors = rospy.get_param('/{}/motors'.format(ROBOT))
expressions = rospy.get_param('/{}/expressions'.format(ROBOT))
animations = rospy.get_param('/{}/animations'.format(ROBOT))

from pau2motors.msg import pau
from pau2motors import ShapekeyStore
rp = RosPack()
config_root = rp.get_path('robots_config')
with open(config_root+"/"+ROBOT+"/motors_settings.yaml", 'r') as stream:
    motors = yaml.load(stream)


msg = pau()
msg.m_shapekeys = [-1.0]*len(ShapekeyStore._shkey_list)

def get_motor_value(motor, relative):
    for m in motors:
        if m['name'] == motor:
            v = m['min']+relative*(m['max'] - m['min'])
            return int(v)

    return -1

def get_cfg(motor):
    global motors
    for m in motors:
        if m['name'] == motor:
            return m
    return None

def weightedsum(cfg,val):
    global msg
    imin = (cfg['init']-cfg['min'])/float(cfg['max']-cfg['min'])
    diff = val - imin
    # Both with same sign
    if (cfg['imax1'] - imin) * diff >= 0:
        im = cfg['imax1']
        mx = cfg['max1']
        ii = 0
    elif (cfg['imax2'] - imin) * diff >= 0:
        im = cfg['imax2']
        mx = cfg['max2']
        ii = 1
    if abs(im - imin) < abs(diff):
        print("Full motor range is not mapped {}".format(cfg['name']))
        diff = im - imin
    sval = abs(diff/float(im-imin))*mx
    # Set values for pay
    keys = cfg['parser_param'].split(";")
    if cfg['parser'] == 'fsshapekey':
        #Check if values not beeing set:
        for i,k in enumerate(keys):
            #print  i, k, ShapekeyStore._current_dict[k], cfg['name']
            if k not in ShapekeyStore._current_dict.keys():
                print("Shapekey not recognized".format(k, ))
                continue
            if msg.m_shapekeys[ShapekeyStore._current_dict[k]] > 0.001:
                print("Shapekey already set {} to {} instead of {}. Motor: {} ".format(k, msg.m_shapekeys[ShapekeyStore._current_dict[k]],  sval, cfg['name']))
                continue
            if i == ii:
                msg.m_shapekeys[ShapekeyStore._current_dict[k]] = sval
            else:
                msg.m_shapekeys[ShapekeyStore._current_dict[k]] = 0

def linear(cfg,val):
    global msg
    sval = cfg['lin_min'] + val * (cfg['lin_max'] - (cfg['lin_min']))
    if cfg['parser'] == 'fsshapekey':
        k = cfg['parser_param']
        if k not in ShapekeyStore._current_dict.keys():
            print("Shapekey {} not recognized".format(k))
            return
        if msg.m_shapekeys[ShapekeyStore._current_dict[k]] > 0.01:
            print("Shapekey already set {} to {} instead of {}. Motor: {} ".format(k, msg.m_shapekeys[ShapekeyStore._current_dict[k]],  sval, cfg['name']))
            return
        msg.m_shapekeys[ShapekeyStore._current_dict[k]] = sval

# Blender methods
def getDriverFromShapekeyName(sk):
    drivers = bpy.data.meshes['Sophia'].shape_keys.animation_data.drivers
    for d in drivers:
        if d.data_path.find(sk) > 0:
            return d
    return None

def getShapekeyName(id):
    if (id >= len(bpy.data.shape_keys['ShapeKeys'].key_blocks)):
        raise Exception("No Shapkey Fond")
    return bpy.data.shape_keys['ShapeKeys'].key_blocks[id].name

def getShapekeyBone(id):
    sk = getShapekeyName(id)
    d = getDriverFromShapekeyName(sk)
    var = None
    if 'var' in d.driver.variables.keys():
        var = d.driver.variables['var']
    elif 'fine' in d.driver.variables.keys():
        var = d.driver.variables['fine']
    exp = d.driver.expression
    p = '+'
    if exp.find('-25') > -1:
        p = '-'
    if not var:
        raise Exception("Driver {} Variable not found".format(sk))
    bone = var.targets[0].bone_target
    return (p,bone)

def newAction(name):
    new_action = bpy.data.actions['0000-neutral'].copy()
    new_action.name = name
    return new_action


def insertkeyframe(action, bone, frame, value, channel = 0):
    k = action.groups[bone].channels[channel].keyframe_points.insert(frame, value)
    return k

def blenderVal(v):
    return v / 25.0

def insertBlenderFrame(anim, sk_id, frame, val):
    p, bone = getShapekeyBone(sk_id)
    val = blenderVal(val)
    if p == '-':
        val = val * -1.0
    insertkeyframe(anim,bone,frame,val)

def importAnimations(animations):
    global msg
    for a in animations:
        current_frame = 1
        aname = list(a.keys())[0]
        if aname != 'happy':
            continue
        frames = list(a.values())[0]
        anim = False
        for f in frames:
            current_frame += f['frames']
            # Reset PAU
            msg = pau()
            msg.m_shapekeys = [-1.0]*len(ShapekeyStore._shkey_list)
            for m,v in f['motors'].items():
                #Get the shapekey and the value
                cfg = get_cfg(m)
                if not cfg:
                    print("No motor config {}".format(m))
                    continue
                if cfg['function'] == 'weightedsum':
                    weightedsum(cfg, v)
                if cfg['function'] == 'linear':
                    linear(cfg, v)
            # Shapekeys calcyulated. Add this to blender.
            print(msg.m_shapekeys)
            for x in range(2, len(msg.m_shapekeys)):
                if msg.m_shapekeys[x] > -0.9:
                    if not anim:
                        anim = newAction("GST-"+aname)
                    try:
                        insertBlenderFrame(anim,x,current_frame,msg.m_shapekeys[x])
                    except Exception as e:
                        print(e)

if __name__ == '__main__':
    importAnimations(animations)
