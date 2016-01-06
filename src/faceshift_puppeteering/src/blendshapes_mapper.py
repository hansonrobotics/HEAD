#!/usr/bin/env python
__author__ = 'tesfa'

import rospy
import rospkg
import json
from dynamic_reconfigure.server import Server
from faceshift_puppeteering.cfg import FBConfig
from blender_api_msgs.msg import FSShapekeys
from blender_api_msgs.msg import FSShapekey


class faceshift_mapper():
    d = {}

    def __init__(self):
        self.parameters = {}
        rospy.init_node("faceshift_puppeteering_mapper", anonymous = True)
        path = rospkg.RosPack().get_path('faceshift_puppeteering')
        shape_key_pairing = '%s/sophia/shapekey_pairing.json' % path
        global d
        with open(shape_key_pairing) as json_data:
            d = json.load(json_data)
        srv = Server(FBConfig, self.callback)
        rospy.Subscriber("/blender_api/faceshift_blendshapes_values", FSShapekeys, self.publisher)
        self.pub=rospy.Publisher('/blender_api/set_shape_keys', FSShapekeys, queue_size=10)
        rospy.spin()
    def callback(self, config, level):
        self.parameters = config
        return config
    def publisher(self,shapekeys):
        dict_shape={}
        for i in shapekeys.shapekey:
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