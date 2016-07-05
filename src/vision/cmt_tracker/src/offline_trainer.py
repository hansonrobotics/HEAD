#!/usr/bin/env python2

import numpy as np
import cv2
import Tkinter as tk
import Image, ImageTk

import rospy
import random
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from cmt_tracker_msgs.msg import Trackers,Tracker,Objects
from cmt_tracker_msgs.srv import TrackerNames
from cmt_tracker_msgs.cfg import RecogntionConfig
from std_srvs.srv import Empty
from sensor_msgs.msg import Image as Imge
from dynamic_reconfigure.server import Server

from openface_wrapper import face_recognizer
import os

class OfflineViewer:

    def __init__(self):
        self.save_faces = False
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

        buttonsFrame = tk.Frame(self.window,width=600, height=20)
        buttonsFrame.grid(row = 600, column=0, padx=10, pady=2)

        liveResultsButton = tk.Button(buttonsFrame,text="Live Results", fg="brown",command= self.switchToLive)
        liveResultsButton.pack(side = tk.LEFT)

        saveFacesButton = tk.Button(buttonsFrame, text="Save Faces", fg="brown", command = self.switchToSave)
        saveFacesButton.pack(side = tk.LEFT)

        trainFacesButton = tk.Button(buttonsFrame,text="Train Faces", fg="brown", command = self.switchToTrain)
        trainFacesButton.pack(side = tk.LEFT)

        trimFacesButton = tk.Button(buttonsFrame, text="Trim Faces", fg="brown", command = self.trimTrainingSet)
        trimFacesButton.pack(side=tk.LEFT)

        #add buttons
        self.window.mainloop()  # Starts GUI

    def switchToLive(self):
        self.save_faces= False

    def switchToTrain(self):
        #Create a dialog for waiting and then when finished say comething.
        self.face_recognizer.train_dataset()
        #Promot to finish the training.

    def trimTrainingSet(self):
        #This one does traininf for the set of images in the system.
        pass

    def switchToSave(self):
        self.save_faces = True

        list_previous = next(os.walk(self.image_dir + '/faces'))[1]

        while(True):
            result = random.randrange(0, 100000, 1)
            if ('temp_' + str(result)) not in list_previous:
                self.generate_unique = 'temp_' + str(result)
                break

    def sample_callback(self, config, level):
        self.back_up = config.image_number
        self.image_sample_size = config.image_number
        self.sample_size = config.sample_size
        return config

    def callback(self, data, face):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            for i in face.objects:
                tupl = []
                for pts in i.feature_point.points:
                    x, y = pts.x, pts.y
                    tupl.append((x, y))
                cv2.rectangle(cv_image,(i.object.x_offset,i.object.y_offset),
                              (i.object.x_offset + i.object.width,i.object.y_offset + i.object.width),(255,0,0))
                #Now let's query every single time to which results group it belongs to and box output the result
                if not self.save_faces:
                    result = self.face_recognizer.immediate_results(cv_image, tupl)
                    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
                    cv2.putText(cv_image,str(result),(i.object.x_offset,i.object.y_offset),font,0.7,(255,0,0),2)
                else:
                    if (self.image_sample_size > 0):
                        self.face_recognizer.save_faces(cv_image,tupl,self.generate_unique,str(self.image_sample_size))
                        self.image_sample_size -= 1
                        break
                    else:
                        self.face_recognizer.train_process(self.generate_unique)
                        self.image_sample_size = self.back_up
                        self.save_faces = False

            cv2image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGBA)

            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.lmain.imgtk = imgtk
            self.lmain.configure(image=imgtk)

        except CvBridgeError as e:
            self.logger.error(e)

if __name__ == '__main__':
    ic = OfflineViewer()