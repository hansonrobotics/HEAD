#!/usr/bin/env python2

import numpy as np
import cv2
import Tkinter as tk
import Image, ImageTk

import rospy

from cv_bridge import CvBridge, CvBridgeError
import message_filters

from cmt_tracker_msgs.msg import Trackers,Tracker,Objects
from cmt_tracker_msgs.srv import TrackerNames
from cmt_tracker_msgs.cfg import RecogntionConfig
from std_srvs.srv import Empty
from sensor_msgs.msg import Image as Imge
from dynamic_reconfigure.server import Server

from openface_wrapper import face_recognizer

class OfflineViewer:

    def __init__(self):
        rospy.init_node('offline_viewer', anonymous=True)
        self.openface_loc = rospy.get_param('openface')
        self.camera_topic = rospy.get_param('camera_topic')
        self.filtered_face_locations = rospy.get_param('filtered_face_locations')
        self.shape_predictor_file = rospy.get_param("shape_predictor")
        self.image_dir = rospy.get_param("image_locations")
        self.face_recognizer = face_recognizer(self.openface_loc, self.image_dir)
        self.tracker_locations_pub = rospy.Publisher("tracking_locations", Trackers, queue_size=5)
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber(self.camera_topic, Imge)
        self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.face_sub],
                                                         10, 0.25)
        ts.registerCallback(self.callback)
        Server(RecogntionConfig, self.sample_callback)
        #Set up GUI
        self.window = tk.Tk()  #Makes main window
        self.window.wm_title("Offline Vizualizer For OpenFace ")
        self.window.config(background="#FFFFFF")

#Graphics window
        imageFrame = tk.Frame(self.window, width=600, height=500)
        imageFrame.grid(row=0, column=0, padx=10, pady=2)

#Capture video frames
        self.lmain = tk.Label(imageFrame)
        self.lmain.grid(row=0, column=0)

        # rospy.spin()
        self.window.mainloop()  # Starts GUI

    def sample_callback(self, config, level):
        self.image_sample_size = config.image_number
        self.sample_size = config.sample_size
        return config

    def callback(self, data, face):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #Let's paint on the picture here.


            for i in face.objects:
                tupl = []

                for pts in i.feature_point.points:
                    x, y = pts.x, pts.y
                    tupl.append((x, y))
                cv2.rectangle(cv_image,(i.object.x_offset,i.object.y_offset),
                              (i.object.x_offset + i.object.width,i.object.y_offset + i.object.width),(255,0,0))
                #Now let's query every single time to which results group it belongs to and box output the result
                result = self.face_recognizer.immediate_results(cv_image, tupl)
                font = cv2.FONT_HERSHEY_COMPLEX_SMALL
                cv2.putText(cv_image,str(result),(i.object.x_offset,i.object.y_offset),font,0.8,(255,0,0),2)
                #print(self.face_recognizer.immediate_results(cv_image, tupl))


            cv2image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGBA)

            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.lmain.imgtk = imgtk
            self.lmain.configure(image=imgtk)

            #Paint and then classify the results as tags.

            #self.lmain.after(10, self.show_frame)
            # Slider window (slider controls stage position)

        except CvBridgeError as e:
            self.logger.error(e)






if __name__ == '__main__':
    ic = OfflineViewer()