# Implements helper functions used by the rest of the module

import math, random

def mix(a,b,factor):
    '''mix two number together using a factor'''
    if type(a) is list or type(a) is tuple:
        if len(a)==len(b)==2:
            return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor)]
        elif len(a)==len(b)==3:
            return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor), a[2]*factor + b[2]*(1.0-factor)]
        elif len(a)==len(b)==4:
            return [a[0]*factor + b[0]*(1.0-factor), a[1]*factor + b[1]*(1.0-factor), a[2]*factor + b[2]*(1.0-factor), a[3]*factor + b[3]*(1.0-factor)]
        else:
            raise Exception(ArithmeticError)
    else:
        return (a*factor + b*(1.0-factor))


def checkValue(var, minV, maxV):
    if not minV <= var <= maxV:
        print('Warning:', var, 'is outside of the expected range ', ':', minV, '-' , maxV)


def smoothstep(x):
    '''returns a smoothed float given an input between 0 and 1'''
    return x * x * (3-2*(x))


def computeDistance(a, b):
    if len(a)==len(b)==2:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    elif len(a)==len(b)==3:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
    elif len(a)==len(b)==4:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2 + (a[3]-b[3])**2)
    else:
        raise Exception(ArithmeticError)


def mapValue(value, leftMin, leftMax, rightMin, rightMax):
    # map value from one range to another range

    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def randomSelect(probs):
    r = random.random()
    index = 0
    while(r >= 0 and index < len(probs)):
        r -= probs[index]
        index += 1
    return index - 1


def m2bu(m, scale=0.25):
    # converts input (assuming centimeters) to Blender Unit:
    # Beorn says in github #25 that one BU should be about 20cm.
    if type(m) is list or type(m) is tuple:
        bu = [i/scale for i in m]
    else:
        bu = m/scale
    return bu

from mathutils import Matrix

def get_pose_matrix_in_other_space(mat, pose_bone, pose_refbone=None):
    """
    Returns the transform matrix relative to pose_bone's current
    transform space. In other words, presuming that mat is in
    armature space, slapping the returned matrix onto pose_bone
    should give it the armature-space transforms of mat.

    If pose_refbone (reference bone) is set, it is used instead of pose_bone's
    parent.

    TODO: try to handle cases with axis-scaled parents better.
    """
    rest = pose_bone.bone.matrix_local.copy()
    rest_inv = rest.inverted()
    if pose_refbone == None and pose_bone.parent:
        pose_refbone = pose_bone.parent
    if pose_refbone:
        par_mat = pose_refbone.matrix.copy()
        par_inv = par_mat.inverted()
        par_rest = pose_refbone.bone.matrix_local.copy()
    else:
        par_mat = Matrix()
        par_inv = Matrix()
        par_rest = Matrix()

    # Get matrix in bone's current transform space
    smat = rest_inv * (par_rest * (par_inv * mat))

    return smat


def get_bones_rotation_rad(armature, bone, axis, refbone=None):
    if refbone:
        mat = get_pose_matrix(
            bpy.data.objects[armature].pose.bones[bone],
            bpy.data.objects[armature].pose.bones[refbone]
            )
    else:
        mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
    return getattr(mat.to_euler(), axis)
