from blender import *
import rospy
import os
from pau2motors.msg import pau
from topic_tools.srv import MuxSelect
import random

#<node pkg="topic_tools" type="mux" name="neck_pau" args="neck_pau cmd_neck_pau /blender_api/get_pau mux:=neck_pau_mux"/>
#<node pkg="topic_tools" type="mux" name="head_pau" args="head_pau no_pau /blender_api/get_pau mux:=head_pau_mux"/>
#<node pkg="topic_tools" type="mux" name="lips_pau" args="lips_pau head_pau lipsync_pau mux:=lips_pau_mux"/>
#<node pkg="topic_tools" type="mux" name="eyes_pau" args="eyes_pau head_pau eyes_tracking_pau mux:=eyes_pau_mux"/>
eye_pau_select = rospy.ServiceProxy("/sophia_body/eyes_pau_mux/select", MuxSelect)
head_pau_select = rospy.ServiceProxy("/sophia_body/head_pau_mux/select", MuxSelect)
neck_pau_select = rospy.ServiceProxy("/sophia_body/neck_pau_mux/select", MuxSelect)
head_pau_select.call("/blender_api/get_pau")
neck_pau_select.call("/blender_api/get_pau")
eye_pau_select.call("eyes_tracking_pau")

pub = rospy.Publisher('/blender_api/get_pau', pau, queue_size=1)

def eyes_calib_send(pitch, yaw, delta):
    if delta > 0.2 or delta < -0.2:
        return
    msg = pau()
    msg.m_eyeGazeLeftPitch = pitch
    msg.m_eyeGazeLeftYaw = yaw
    msg.m_eyeGazeRightPitch = pitch
    msg.m_eyeGazeRightYaw = yaw
    msg.m_eyeGazeLeftPitch += delta
    msg.m_eyeGazeRightPitch += delta
    msg.m_eyeGazeLeftYaw += delta
    msg.m_eyeGazeRightYaw += delta

    pub.publish(msg)
    print "pub {}".format(msg)

def eyes_calib():
    set_alive(False)
    set_blink_randomly(False)
    set_saccade(False)
    rospy.init_node('calibration')
    r = rospy.Rate(1)

    os.system('rosrun dynamic_reconfigure dynparam set /sophia_body/eye_tracking tracking True')

    delta = 0
    while not rospy.is_shutdown():
        r.sleep()
        eyes_calib_send(pitch=0.2, yaw=0, delta=delta)
        delta = -1*delta

    #os.system('rosrun dynamic_reconfigure dynparam set /sophia_body/eye_tracking tracking False')

keys = ['Basis', 'Shrinkwrap', 'adjustments', 'brow_center_UP', 'brow_center_DN', 'brow_inner_UP.L', 'brow_inner_DN.L', 'brow_inner_UP.R', 'brow_inner_DN.R', 'brow_outer_UP.L', 'brow_outer_DN.L', 'brow_outer_up.R', 'brow_outer_DN.R', 'eye-flare.UP.L', 'eye-blink.UP.L', 'eye-flare.UP.R', 'eye-blink.UP.R', 'eye-blink.LO.L', 'eye-flare.LO.L', 'eye-blink.LO.R', 'eye-flare.LO.R', 'wince.L', 'wince.R', 'sneer.L', 'sneer.R', 'eyes-look.dn', 'eyes-look.up', 'lip-UP.C.UP', 'lip-UP.C.DN', 'lip-UP.L.UP', 'lip-UP.L.DN', 'lip-UP.R.UP', 'lip-UP.R.DN', 'lips-smile.L', 'lips-smile.R', 'lips-wide.L', 'lips-narrow.L', 'lips-wide.R', 'lips-narrow.R', 'lip-DN.C.DN', 'lip-DN.C.UP', 'lip-DN.L.DN', 'lip-DN.L.UP', 'lip-DN.R.DN', 'lip-DN.R.UP', 'lips-frown.L', 'lips-frown.R', 'lip-JAW.DN', 'jaw']

def brows_calib(msg):
    keys = get_shape_keys()
    values =msg.m_coeffs
    data = {k:v for k, v in zip(keys, values)}

    print [data['lip-UP.L.UP'], data['lip-UP.L.DN'], data['lip-UP.C.UP'], data['lip-UP.C.DN'], data['lip-UP.R.UP'], data['lip-UP.R.DN']]

    #pitch = [msg.m_eyeGazeLeftPitch, msg.m_eyeGazeRightPitch]
    #yaw = [msg.m_eyeGazeLeftYaw, msg.m_eyeGazeRightYaw]
    #print yaw+pitch


if __name__ == '__main__':
    rospy.init_node("calibration")
    rospy.Subscriber('/blender_api/get_pau', pau, brows_calib)
    rospy.spin()

