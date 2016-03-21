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
    p = 1
    if exp.find('-25') > -1:
        p = -1
    if not var:
        eyelids = {
            'eye-flare.UP.L': [1, 'eyelid_UP_L'],
            'eye-blink.UP.L': [-1, 'eyelid_UP_L'],
            'eye-flare.UP.R': [1, 'eyelid_UP_R'],
            'eye-blink.UP.R': [-1, 'eyelid_UP_R'],
            'eye-flare.LO.L': [1, 'eyelid_LO_L'],
            'eye-blink.LO.L': [-1, 'eyelid_LO_L'],
            'eye-flare.LO.R': [1, 'eyelid_LO_R'],
            'eye-blink.LO.R': [-1, 'eyelid_LO_R'],
        }
        if sk in eyelids.keys():
            return eyelids[sk]
        raise Exception("Sk {} Variable not found".format(sk))
    else:
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
    val = val * p
    insertkeyframe(anim,bone,frame,val)

def getKeyFrameFromPAU(pau):
    bone_values = {}
    # All shapekey values summed for face motors
    for x in range(2, len(pau.m_shapekeys)):
        if msg.m_shapekeys[x] > -0.9:
            try:
                p, bone = getShapekeyBone(x)
            except Exception as e:
                print(e)
            val = blenderVal(msg.m_shapekeys[x])
            val = val * p
            if bone in bone_values.keys():
                bone_values[bone] += val
            else:
                bone_values[bone] = val
    return bone_values

def getPauFromMotors(motors):
    global msg
    msg = pau()
    msg.m_shapekeys = [-1.0]*len(ShapekeyStore._shkey_list)
    for m,v in motors.items():
        #Get the shapekey and the value
        cfg = get_cfg(m)
        if not cfg:
            print("No motor config {}".format(m))
            continue
        if cfg['function'] == 'weightedsum':
            weightedsum(cfg, v)
        if cfg['function'] == 'linear':
            linear(cfg, v)
    return msg

def findAction(act):
    if act not in bpy.data.actions.keys():
        return False
    else:
        return bpy.data.actions[act]

def importAnimations(animations):
    global msg
    for a in animations:
        current_frame = 1
        aname = list(a.keys())[0]
        frames = list(a.values())[0]
        anim = False
        try:
            for f in frames:
                current_frame += f['frames']
                # Reset PAU
                m = getPauFromMotors(f['motors'])
                kf = getKeyFrameFromPAU(m)
                for bone, val in kf.items():
                    if not anim:
                        anim = newAction("GST-"+aname)
                    try:
                        insertkeyframe(anim,bone,current_frame, val)
                    except Exception as e:
                        print(e)
        except Exception as e:
            print("Cannot import {} with error {}".format(aname, str(e)))

def removeKeyFrames(action, bone, start, finish, channel=0):
        remove = []
        points = action.groups[bone].channels[0].keyframe_points.values()
        for kf in points:
            if start <= kf.co[0] <= finish:
                remove.append(kf)
        for kf in remove:
            action.groups[bone].channels[0].keyframe_points.remove(kf)

def findExpression(exp):
    return findAction('EMO-'+exp.lower())

def findVisime(vis):
    return findAction('VIS-'+vis)

def updateExpressions(expressions):
    for e in expressions:
        exp = list(e.keys())[0]
        motors = list(e.values())[0]
        visime = False
        if exp.find('vis_') > -1:
            action = findVisime(exp[4:])
            if not action:
                print("Skipping Visime "+exp[4:])
                continue
            visime = True
        else:
            action = findExpression(exp)
            if not action:
                action = newAction('EMO-'+exp.lower())
        m = getPauFromMotors(motors)
        kf = getKeyFrameFromPAU(m)
        for bone, val in kf.items():
            # Need to clear previous KF frames first
            try:
                if not visime:
                    # All expressions has 101 frame as max
                    removeKeyFrames(action, bone, 2,102)
                    insertkeyframe(action, bone, 101, val)
                else:
                    removeKeyFrames(action, bone, 1,2)
                    insertkeyframe(action, bone, 1, val)
            except Exception as ex:
                print(ex)

if __name__ == '__main__':
    importAnimations(animations)
#    updateExpressions(expressions)

