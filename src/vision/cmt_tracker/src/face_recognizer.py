#!/usr/bin/env python2
import os
import sys
import subprocess
import rospy
import cv2
import logging
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from cmt_tracker_msgs.msg import Trackers,Tracker,Objects
from cmt_tracker_msgs.srv import TrackerNames
from cmt_tracker_msgs.cfg import TrackerConfig
from dynamic_reconfigure.server import Server

import numpy as np
import rospkg
import pickle
import pandas as pd
import openface
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from operator import itemgetter
from collections import defaultdict
import shutil

TEMPLATE = np.float32([
    (0.0792396913815, 0.339223741112), (0.0829219487236, 0.456955367943),
    (0.0967927109165, 0.575648016728), (0.122141515615, 0.691921601066),
    (0.168687863544, 0.800341263616), (0.239789390707, 0.895732504778),
    (0.325662452515, 0.977068762493), (0.422318282013, 1.04329000149),
    (0.531777802068, 1.06080371126), (0.641296298053, 1.03981924107),
    (0.738105872266, 0.972268833998), (0.824444363295, 0.889624082279),
    (0.894792677532, 0.792494155836), (0.939395486253, 0.681546643421),
    (0.96111933829, 0.562238253072), (0.970579841181, 0.441758925744),
    (0.971193274221, 0.322118743967), (0.163846223133, 0.249151738053),
    (0.21780354657, 0.204255863861), (0.291299351124, 0.192367318323),
    (0.367460241458, 0.203582210627), (0.4392945113, 0.233135599851),
    (0.586445962425, 0.228141644834), (0.660152671635, 0.195923841854),
    (0.737466449096, 0.182360984545), (0.813236546239, 0.192828009114),
    (0.8707571886, 0.235293377042), (0.51534533827, 0.31863546193),
    (0.516221448289, 0.396200446263), (0.517118861835, 0.473797687758),
    (0.51816430343, 0.553157797772), (0.433701156035, 0.604054457668),
    (0.475501237769, 0.62076344024), (0.520712933176, 0.634268222208),
    (0.565874114041, 0.618796581487), (0.607054002672, 0.60157671656),
    (0.252418718401, 0.331052263829), (0.298663015648, 0.302646354002),
    (0.355749724218, 0.303020650651), (0.403718978315, 0.33867711083),
    (0.352507175597, 0.349987615384), (0.296791759886, 0.350478978225),
    (0.631326076346, 0.334136672344), (0.679073381078, 0.29645404267),
    (0.73597236153, 0.294721285802), (0.782865376271, 0.321305281656),
    (0.740312274764, 0.341849376713), (0.68499850091, 0.343734332172),
    (0.353167761422, 0.746189164237), (0.414587777921, 0.719053835073),
    (0.477677654595, 0.706835892494), (0.522732900812, 0.717092275768),
    (0.569832064287, 0.705414478982), (0.635195811927, 0.71565572516),
    (0.69951672331, 0.739419187253), (0.639447159575, 0.805236879972),
    (0.576410514055, 0.835436670169), (0.525398405766, 0.841706377792),
    (0.47641545769, 0.837505914975), (0.41379548902, 0.810045601727),
    (0.380084785646, 0.749979603086), (0.477955996282, 0.74513234612),
    (0.523389793327, 0.748924302636), (0.571057789237, 0.74332894691),
    (0.672409137852, 0.744177032192), (0.572539621444, 0.776609286626),
    (0.5240106503, 0.783370783245), (0.477561227414, 0.778476346951)])

TPL_MIN, TPL_MAX = np.min(TEMPLATE, axis=0), np.max(TEMPLATE, axis=0)
MINMAX_TEMPLATE = (TEMPLATE - TPL_MIN) / (TPL_MAX - TPL_MIN)


class face_recognizer:
  def __init__(self):
    #This is one is models of network that is not used in this node
    rospy.init_node('face_recognizer', anonymous=True)
    self.inner_eyes_and_bottom_lip = [38,42,57]

    #This is one that is used for trained model nn4.small2
    self.outer_eyes_and_nose = [36, 45, 33]

    #Are there better network models other that this????
    self.openface_loc = rospy.get_param('openface')
    self.net = openface.TorchNeuralNet(self.openface_loc + '/models/openface/nn4.small2.v1.t7',imgDim=96,cuda=False)

    self.camera_topic = rospy.get_param('camera_topic')
    self.filtered_face_locations = rospy.get_param('filtered_face_locations')
    self.shape_predictor_file = rospy.get_param("shape_predictor")


    self.image_dir = rospy.get_param("image_locations")
    self.image_dir_face_imgs = self.image_dir + "/faces"

    #TODO remove all the temp images here to start up things.
    self.image_dir_face_temp = self.image_dir + "/tmp"

    # Delete all the items in temp folder so as not to consume space. May be when it's brought down we can delete them. or in destructor without sigterm.
    for the_file in os.listdir(self.image_dir_face_temp):
        file_path = os.path.join(self.image_dir_face_temp, the_file)
        shutil.rmtree(file_path)

    self.feature_dir = self.image_dir + "/feature"

    if (not os.path.exists(self.image_dir_face_imgs)):
        os.makedirs(self.image_dir_face_imgs)
    if (not os.path.exists(self.feature_dir)):
        os.makedirs(self.feature_dir)
    self.tracker_locations_pub = rospy.Publisher("tracking_locations",Trackers,queue_size=5)

    #Get Environmental variable for the file during start up.
    self.batch_represent = self.openface_loc + '/batch-represent/main.lua'

    #print(self.batch_represent)
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber(self.camera_topic, Image)
    self.cmt_sub = message_filters.Subscriber('tracker_results',Trackers)
    self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)
    ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cmt_sub,self.face_sub], 10,0.5)

    ts.registerCallback(self.callback)
    Server(TrackerConfig, self.sample_callback)
    self.faces_cmt_overlap = {}
    self.faces_recognized_lap = {}

    self.logger = logging.getLogger('hr.cmt_tracker.face_recognizer_node')
  def callback(self,data, cmt, face):
    self.logger.debug('Overlap on face and tracker')
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      self.logger.error(e) # This need to handle the possibility of errors.

    # Now let's calculate the area of the elements.
    not_covered_faces,covered_faces = self.returnOverlapping(face,cmt)
    
    # Not Covered Faces
    if len(not_covered_faces) > 0:
        self.tracker_locations_pub.publish(self.convert(not_covered_faces))
        rospy.set_param('tracker_updated', 2)
        rospy.set_param("being_initialized_stop",1)
        print("New Faces to be added to tracker: " + str(len(not_covered_faces)))
    # Covered Faces Check:
    for face,cmt in covered_faces:
        tupl = []
        for pts in face.feature_point.points:
            x,y = pts.x, pts.y
            tupl.append((x,y))

        self.faces_cmt_overlap[cmt.tracker_name.data] = self.faces_cmt_overlap.get(cmt.tracker_name.data,0) + 1

        if self.faces_cmt_overlap[cmt.tracker_name.data] < self.sample_size and not cmt.recognized.data: # The first 10 images
            print('going to the results')

            if os.path.exists("{}/classifier.pkl".format(self.feature_dir)):
                self.results(cv_image,tupl,cmt)
            else:
                #TODO remove eliminates.
                self.faces_cmt_overlap[cmt.tracker_name.data] = 10
            #TODO save faces here also to avoid going 60 faces into the future.

            print('coming back to callback')

        elif self.faces_cmt_overlap[cmt.tracker_name.data] == self.sample_size and not cmt.recognized.data:
            print('assigning faces. ')
            print(self.faces_recognized_lap)
            max_index = max(self.faces_recognized_lap[cmt.tracker_name.data]['results'].iterkeys(),
                            key=(lambda key: self.faces_recognized_lap[cmt.tracker_name.data]['results']))
            try:
                self.upt = rospy.ServiceProxy('recognition',TrackerNames)
                indication = self.upt(names=cmt.tracker_name.data, index=int(max_index[5:]))
                if not indication:
                    self.logger.warn("there was the same id in the id chamber. Training again....")
            except rospy.ServiceException, e:
                self.logger.error("Service call failed: %s" %e)
        elif self.faces_cmt_overlap[cmt.tracker_name.data] < (self.image_sample_size + self.sample_size) and not cmt.recognized.data:
            if (not os.path.exists(self.image_dir_face_temp + "/" + cmt.tracker_name.data)):
                os.makedirs(self.image_dir_face_temp + "/" + cmt.tracker_name.data)
            print('saving faces')
            self.save_faces(cv_image,tupl,self.image_dir_face_temp + "/" + cmt.tracker_name.data + "/" + str(self.faces_cmt_overlap[cmt.tracker_name.data]))
            print('finished saving faces')
        ## TODO this when there is a new file in the dataset.

        elif self.faces_cmt_overlap[cmt.tracker_name.data] == self.image_sample_size + self.sample_size:
            #TODO Plot the process of this dataset indicated https://github.com/cmusatyalab/openface/blob/master/training/plot-loss.py#L72
            print('training the process')
            # THIS updates the entire training set. This should set everything to be queried in again.
            self.train_process(cmt)
            print('finished training the model.')

            #TODO behaviour scripts accounting.

        else:
            #TODO this is where we occasionally check results to check for results.

            #TODO check's first when it get's into the system. So we have to make the system do change the names at first when training is done.

            #TODO otherwise how does it take new names from people in a way.
            pass

  def sample_callback(self,config, level):
      self.image_sample_size = config.image_number
      self.sample_size = config.sample_size
      return config

  def train_process(self, cmt):
      print('starts training')
      subprocess.call(['mv', self.image_dir_face_temp + "/" + cmt.tracker_name.data,
                       self.image_dir_face_imgs + "/" + cmt.tracker_name.data])
      # TO avoid calling training if the faces directory doesn't have faces at least two faces.
      if len(list(os.walk(self.image_dir_face_imgs).next()[1])) > 1:
          # As at this point the dataset has changed thusly we have to delete the cache.
          if (os.path.exists(self.image_dir_face_imgs + "/" + 'cache.t7')):
              subprocess.call(['rm', self.image_dir_face_imgs + "/" + 'cache.t7'])
              # Here call the batch represent file which is a lua function in a process of it's own.
          subprocess.call([self.batch_represent, '-outDir', self.feature_dir, '-data', self.image_dir_face_imgs])
          # here the number of images in the classifier must be greater than 1
          self.train()
      print('finsihes training')

  def returnOverlapping(self, face, cmt):
    print('starts overlapping')
    not_covered_faces = []
    overlaped_faces = []
    for j in face.objects:
        overlap = False
        SA = j.object.height * j.object.width
        for i in cmt.tracker_results:
            SB = i.object.object.height * i.object.object.width
            SI = max(0, ( max(j.object.x_offset + j.object.width,i.object.object.x_offset + i.object.object.width)- min(j.object.x_offset,i.object.object.x_offset) )
                     * max(0,max(j.object.y_offset,i.object.object.y_offset) - min(j.object.y_offset - j.object.height,i.object.object.y_offset - i.object.object.height)))
            SU = SA + SB - SI
            overlap_area = SI / SU
            overlap = overlap_area > 0
            if (overlap):
                ## TODO DO we need to remove the tracker that was indeed needed in the element.
                list = [j,i]
                overlaped_faces.append(list)
                break
        if not overlap:
            not_covered_faces.append(j)
    print('finishes overlapping')
    return not_covered_faces, overlaped_faces
  def convert(self, face_locs):
    print('reaches convert')
    message = Trackers()
    for i in face_locs:
        messg = Tracker()
        messg.object = i
        message.tracker_results.append(messg)
    message.header.stamp = rospy.Time.now()
    print('finishes convert')
    return message

  def save_faces(self,cv_image, tupl, loc_save):
      #TODO include varing alignment parameters.
      img_aligned = self.align(cv_image,tupl)
      cv2.imwrite( loc_save +".jpg",img_aligned)

  def results(self,cv_image,tupl,i,threshold=0.85):
    print('reaches results')
    #TODO Even take out this existence
    if os.path.exists("{}/classifier.pkl".format(self.feature_dir)):
        img_aligned = self.align(cv_image,tupl,True)
        result = self.infer(img_aligned)
        self.faces_recognized_lap[i.tracker_name.data] = self.faces_recognized_lap.get(i.tracker_name.data, {})
        self.faces_recognized_lap[i.tracker_name.data]['results'] = self.faces_recognized_lap[i.tracker_name.data].get('results',{})
        self.faces_recognized_lap[i.tracker_name.data]['results'][result[0]] = self.faces_recognized_lap[i.tracker_name.data]['results'].get(result[0],0)

        if result[1] > threshold: ##TODO Dynamic reconfigurable name; if's below this threshold then stop.
            self.faces_recognized_lap[i.tracker_name.data]['results'][result[0]] = self.faces_recognized_lap[i.tracker_name.data]['results'][result[0]] + 1
        else:
            self.faces_recognized_lap[i.tracker_name.data]['results'][result[0]] = self.faces_recognized_lap[i.tracker_name.data]['results'][result[0]] + 0
    print('finishes result')

  def align(self, img, landmarks,net=False):
    npLandmarks = np.float32(landmarks)
    npLandmarksIndices = np.array(self.outer_eyes_and_nose)
    H = cv2.getAffineTransform(npLandmarks[npLandmarksIndices],
                                   96 * MINMAX_TEMPLATE[npLandmarksIndices])
    thumbnail = cv2.warpAffine(img, H, (96, 96))
    if (net):
        thumbnail = self.net.forward(thumbnail).reshape(1,-1)
        return thumbnail
    return thumbnail

  def train(self):
    print('Training Dataset')
    fname = "{}/labels.csv".format(self.feature_dir)
    labels = pd.read_csv(fname, header=None).as_matrix()[:, 1]
    labels = map(itemgetter(1),
                     map(os.path.split,
                         map(os.path.dirname, labels)))  # Get the directory.
    fname = "{}/reps.csv".format(self.feature_dir)
    embeddings = pd.read_csv(fname, header=None).as_matrix()
    le = LabelEncoder().fit(labels)
    labelsNum = le.transform(labels)

    svm = SVC(C=1, kernel='linear', probability=True).fit(embeddings, labelsNum)
    with open("{}/classifier.pkl".format(self.feature_dir), 'w') as f:
        pickle.dump((le, svm), f)
    print('Finished Training')

  def infer(self,net_forwarded_img):
    print('Inferring Result')
    with open("{}/classifier.pkl".format(self.feature_dir), 'r') as f:
        (le, svm) = pickle.load(f)

    prediction = svm.predict_proba(net_forwarded_img)[0]
    maxI = np.argmax(prediction)
    person = le.inverse_transform(maxI)
    confidence = prediction[maxI]
    print('Finished Inferring')
    return (person,confidence)
    
if __name__ == '__main__':
    ic = face_recognizer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        logging.warn("Shutting down")


