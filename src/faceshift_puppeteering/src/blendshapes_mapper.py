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
from blender_api_msgs.msg import AnimationMode
from pau2motors.MapperFactory import Quaternion2EulerYZX
from std_msgs.msg import Float32


class faceshift_mapper():
    d = {}

    def __init__(self):
        self.parameters = {}
        # Current modes for publisher that could be combined by addition
        self.modes={
            'head_yaw': 1,
            'head_pitch': 2, # Currently not supprted by API to control yaw and pitch separately
            'head_roll': 4,
            'eyes': 8,
            'face': 16,
        }
        rospy.init_node("faceshift_puppeteering_mapper", anonymous = True)
        path = rospkg.RosPack().get_path('faceshift_puppeteering')
        shape_key_pairing = '%s/sophia/shapekey_pairing.json' % path
        global d
        with open(shape_key_pairing) as json_data:
            d = json.load(json_data)
        srv = Server(FBConfig, self.callback)
        rospy.Subscriber("/blender_api/faceshift_values", FSValues, self.publisher)

        self.pub_setter= rospy.Publisher('/blender_api/set_animation_mode', AnimationMode, queue_size=10)
        self.pub=rospy.Publisher('/blender_api/set_shape_keys', FSShapekeys, queue_size=10)
        self.pub_neck= rospy.Publisher('/blender_api/set_face_target', Target, queue_size=10)
        self.pub_head_rot = rospy.Publisher('/blender_api/set_head_rotation', Float32, queue_size=10)
        self.pub_gaze= rospy.Publisher('/blender_api/set_gaze_target', Target, queue_size=10)

        self.quoternion2euler = {
            'p': Quaternion2EulerYZX({'axis': 'x'}, None),
            'y': Quaternion2EulerYZX({'axis': 'y'}, None),
            'r': Quaternion2EulerYZX({'axis': 'z'}, None),
        }
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
        
        mode = rospy.get_param("/fb_mode")

        eyes_move = Target()
        head_move= Target()
        b= AnimationMode()


        if mode != 0:
            b.value= 1
        else:
            b.value= 0
            #Now here we set the eye and head targets to the neutral values. The shapekeys are reverted back in the blender_api.
            eyes_move.x=1
            eyes_move.y=0
            eyes_move.z=0

            head_move.x=1
            head_move.y=0
            head_move.z=0

            self.pub_neck.publish(head_move)
            self.pub_gaze.publish(eyes_move)

        self.pub_setter.publish(b)
        # For publishing the head pose.

        # For publishing the Neck Pose.
        pitch = 0
        yaw = 0
        if mode & (self.modes['head_yaw'] | self.modes['head_roll']):
            q = shapekeys.head_pose.orientation
            try:
                pitch = self.quoternion2euler['p'].map(q)
                yaw = self.quoternion2euler['y'].map(q)
                roll = self.quoternion2euler['r'].map(q)
            except:
                pitch = 0
                yaw = 0
                roll = 0
            az = math.sin(pitch)
            ay = math.sin(yaw)*math.cos(pitch)
            # Target one meter away
            ax = math.cos(yaw)*math.cos(pitch)
            head_move.y = ay
            head_move.x = ax
            # Inverted Z for blender
            head_move.z = -az
            if mode & self.modes['head_yaw']:
                self.pub_neck.publish(head_move)
            if mode & self.modes['head_roll']:
                self.pub_head_rot.publish(roll)

        # TODO neads to be fixed. Current blender API only accepts the eye target global position
        # Need to have option in API to set relative to the current head position
        if mode & self.modes['eyes']:
            pitch += math.radians(shapekeys.eye_left.x)
            yaw += math.radians(shapekeys.eye_left.y)
            az = math.sin(pitch)
            ay = math.sin(yaw)*math.cos(pitch)
            # Target one meter away
            ax = math.cos(yaw)*math.cos(pitch)
            eyes_move.x = ax
            eyes_move.y = ay
            eyes_move.z = az
            self.pub_gaze.publish(eyes_move)

        if mode & self.modes['face']:
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
