#!/usr/bin/env python
import math

__author__ = 'tesfa'

import rospy
import rospkg
import json
from dynamic_reconfigure.server import Server
from faceshift_puppeteering.cfg import FBConfig
from blender_api_msgs.msg import FSValues
from blender_api_msgs.msg import FSShapekeys
from blender_api_msgs.msg import FSShapekey
from blender_api_msgs.msg import Target


class faceshift_mapper():
    d = {}

    def __init__(self):
        self.parameters = {}
        self.bitmasks={"no":"000", "face":"001","neck":"010","eyes":"100",'all':"111"}
        rospy.init_node("faceshift_puppeteering_mapper", anonymous = True)
        path = rospkg.RosPack().get_path('faceshift_puppeteering')
        shape_key_pairing = '%s/sophia/shapekey_pairing.json' % path
        global d
        with open(shape_key_pairing) as json_data:
            d = json.load(json_data)
        srv = Server(FBConfig, self.callback)
        rospy.Subscriber("/blender_api/faceshift_values", FSValues, self.publisher)
        self.pub=rospy.Publisher('/blender_api/set_shape_keys', FSShapekeys, queue_size=10)
        self.pub_neck= rospy.Publisher('/blender_api/set_face_target', Target, queue_size=10)
        self.pub_gaze= rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=10)
        rospy.spin()
    def callback(self, config, level):
        self.parameters = config
        return config
    def publisher(self,shapekeys):
        '''
        :param shapekeys: Holds the values of the FSSvalue which holds the head pose(translation and rotation(in quatrenion) and eye_left and right with vector3 data for gaze and FSShapekeys to hold
         the blendhsape values from the system.
        :return:
        '''
        dict_shape={}

        #mode= rospy.get_param("~fb_animation_mode")


        # For publishing the head pose.

        # For publishing the Neck Pose.
        head_move= Target()

        #head_move.x = shapekeys.head_pose.orientation.x / math.sin(math.acos(shapekeys.head_pose.orientation.w))
        #head_move.y = shapekeys.head_pose.orientation.y / math.sin(math.acos(shapekeys.head_pose.orientation.w))
        #head_move.z = shapekeys.head_pose.orientation.z / math.sin(math.acos(shapekeys.head_pose.orientation.w))

        x= shapekeys.head_pose.orientation.x
        y= shapekeys.head_pose.orientation.y
        z= shapekeys.head_pose.orientation.z
        w= shapekeys.head_pose.orientation.w

        head_move.y = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
        #head_move.x = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
        #head_move.z =  math.asin(2*x*y + 2*z*w)

        self.pub_neck.publish(head_move)

        # eye_move_left= Target()
        # eye_move_left.x= shapekeys.eye_left.x
        # eye_move_left.y= shapekeys.eye_left.y
        # eye_move_left.z= shapekeys.eye_left.z
        #
        # #self.pub_gaze.publish(eye_move_left)
        #
        # eye_move_right= Target()
        # eye_move_right.x= shapekeys.eye_right.x
        # eye_move_right.y= shapekeys.eye_right.y
        # eye_move_right.z= shapekeys.eye_right.z


        # For iterating over the shapekeys
        for i in shapekeys.keys.shapekey:
            dict_shape[i.name]= i.value

        publish_mesg = FSShapekeys()

        if (bool(self.parameters)):
            for i in d:
                fshift_name = i
                pub_shape= FSShapekey()

                value = 0
                name= i.replace("-","").replace(".","")
                values= self.parameters['groups']['groups'][name]['parameters']

                #Check a list of items.
                if bool(values):
                    pub_shape.name= fshift_name
                    for parameter in values:
                        sk= parameter.replace(name+"_","")
                        # this below doesn't work as it is the past values. The current values is
                        # value = value + (values[parameter]) * dict_shape[sk]
                        # if 'JAW' in fshift_name:
                           # rospy.loginfo(values[parameter])
                        value = value + (self.parameters[parameter]) * dict_shape[sk]
                        pub_shape.value= value

                    publish_mesg.shapekey.append(pub_shape)
            self.pub.publish(publish_mesg)
        else:
            rospy.loginfo("Problem")


if __name__ == "__main__":
    mapper= faceshift_mapper()