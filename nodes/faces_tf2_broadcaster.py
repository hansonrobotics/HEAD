#!/usr/bin/env python
import rospy
import tf2_ros
from pi_face_tracker.msg import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import time

broadcasters = {}

def handle_faces(msg):
    f= Faces()

    for i, face in enumerate(msg.faces):
        if not i in broadcasters.keys():
            broadcasters[i] =  tf2_ros.TransformBroadcaster()
        transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="camera"),
                             transform=Transform(translation=Vector3(face.point.x,face.point.y,face.point.z),
                                                 rotation=Quaternion(0,0,0,1)),
                             child_frame_id="Face" + str(face.id)
                            )
        broadcasters[i].sendTransform(transform)
        time.sleep(0.01) # Need to investigate why we cant publish multiple messages at same time

if __name__ == '__main__':
    rospy.init_node('faces_tf2_broadcaster')
    rospy.Subscriber('/faces3d',
                     pi_face_tracker.msg.Faces,
                     handle_faces)
    rospy.spin()