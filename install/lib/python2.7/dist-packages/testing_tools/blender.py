import rospy
from blender_api_msgs.srv import SetParam, GetParam

set_param = rospy.ServiceProxy('/blender_api/set_param', SetParam)
get_param = rospy.ServiceProxy('/blender_api/get_param', GetParam)

def set_alive(alive):
    """
    alive:
        True, keep model alive
        False, the opposite
    """
    set_param("bpy.context.scene['keepAlive']", str(alive))

def set_blink_randomly(b):
    set_param("bpy.data.scenes['Scene'].actuators.ACT_blink_randomly.HEAD_PARAM_enabled", str(b))

def set_saccade(b):
    set_param("bpy.data.scenes['Scene'].actuators.ACT_saccade.HEAD_PARAM_enabled", str(b))

def set_command_listener_active(active):
    """
    active:
        True, wm.command_listener is not publishing PAU message
        False, the opposite
    """
    set_param("bpy.context.scene['commandListenerActive']", str(alive))

def get_shape_keys():
    from collections import OrderedDict
    shape_keys = eval(get_param("self.getFaceData()").value).keys()
    return shape_keys

