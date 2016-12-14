#!/usr/bin/env python2
#
# Copyright 2015-2016 Carnegie Mellon University
# Copyright 2016 Hanson Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import cv2
import pickle
import random
import numpy as np
import pandas as pd
np.set_printoptions(precision=2)
import logging

from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
import openface
from openface.data import iterImgs
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from face_recognition.cfg import FaceRecognitionConfig

HR_MODELS = os.environ.get('HR_MODELS', os.path.expanduser('~/.hr/cache/models'))
DLIB_FACEPREDICTOR = os.path.join(HR_MODELS, 
                    'shape_predictor_68_face_landmarks.dat')
NETWORK_MODEL = os.path.join(HR_MODELS, 'nn4.small2.v1.t7')
CLASSIFIER_MODEL = os.path.join(HR_MODELS, 'classifier.pkl')
logger = logging.getLogger('hr.vision.face_recognition.face_recognizer')

class FaceRecognizer(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.imgDim = 96
        self.align = openface.AlignDlib(DLIB_FACEPREDICTOR)
        self.net = openface.TorchNeuralNet(NETWORK_MODEL, self.imgDim)
        self.landmarkIndices = openface.AlignDlib.OUTER_EYES_AND_NOSE
        with open(CLASSIFIER_MODEL, 'r') as f:
            self.le, self.clf = pickle.load(f)
        self.data_root = 'faces'
        self.data_dir = os.path.join(self.data_root, 'training-images')
        #self.output_dir = tempfile.mkdtemp(dir=output_dir)
        if not os.path.isdir(self.data_dir):
            os.makedirs(self.data_dir)


    def getRep(self, bgrImg, all=True):
        if bgrImg is None:
            return

        rgbImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2RGB)

        if all:
            bb = self.align.getAllFaceBoundingBoxes(rgbImg)
        else:
            bb = self.align.getLargestFaceBoundingBox(rgbImg)

        if bb is None:
            return

        if not hasattr(bb, '__iter__'):
            bb = [bb]

        alignedFaces = []
        for box in bb:
            alignedFaces.append(
                self.align.align(
                    self.imgDim,
                    rgbImg,
                    box,
                    landmarkIndices=self.landmarkIndices))

        reps = []
        for alignedFace in alignedFaces:
            reps.append(self.net.forward(alignedFace))

        return reps

    def align_images(self, input_dir, output_dir):
        imgs = list(iterImgs(input_dir))
        # Shuffle so multiple versions can be run at once.
        random.shuffle(imgs)

        for imgObject in imgs:
            print "=== {} ===".format(imgObject.path)
            outDir = os.path.join(output_dir, imgObject.cls)
            if not os.path.isdir(outDir):
                os.makedirs(outDir)
            outputPrefix = os.path.join(outDir, imgObject.name)
            imgName = outputPrefix + ".png"

            if os.path.isfile(imgName):
                print "  + Already found, skipping."
            else:
                rgb = imgObject.getRGB()
                if rgb is None:
                    outRgb = None
                else:
                    outRgb = self.align.align(
                            self.imgDim, rgb,
                            landmarkIndices=self.landmarkIndices)
                if outRgb is not None:
                    outBgr = cv2.cvtColor(outRgb, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(imgName, outBgr)
                    print "  + Writing aligned file to disk."

    def prepare(self):
        output_dir = os.path.join(self.data_root, 'aligned-images')
        self.align_images(self.data_dir, output_dir)

        label_fname = "{}/labels.csv".format(output_dir)
        reps_fname = "{}/reps.csv".format(output_dir)
        for imgObject in iterImgs(output_dir):
            reps = self.net.forward(imgObject.getRGB())

    def train(self):
        self.prepare()
        label_fname = "{}/labels.csv".format(self.data_dir)
        reps_fname = "{}/reps.csv".format(self.data_dir)
        labels = pd.read_csv(label_fname, header=None).as_matrix()[:, 1]
        labels = map(itemgetter(1),
                    map(os.path.split,
                        map(os.path.dirname, labels)))  # Get the directory.
        embeddings = pd.read_csv(reps_fname, header=None).as_matrix()
        le = LabelEncoder().fit(labels)
        labelsNum = le.transform(labels)

        clf = SVC(C=1, kernel='linear', probability=True)
        clf.fit(embeddings, labelsNum)

        classifier_fname = "{}/classifier.pkl".format(self.data_dir)
        with open(classifier_fname, 'w') as f:
            pickle.dump((le, clf), f)
        logger.info("Model saved to {}".format(classifier_fname))

    def infer(self, img):
        reps = self.getRep(img)
        persons = []
        confidences = []
        for rep in reps:
            try:
                rep = rep.reshape(1, -1)
            except:
                logger.info("No Face detected")
                return (None, None)
            predictions = self.clf.predict_proba(rep).ravel()
            maxI = np.argmax(predictions)
            persons.append(self.le.inverse_transform(maxI))
            confidences.append(predictions[maxI])
        return (persons, confidences)

    def image_cb(self, ros_image):
        image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        persons, confidences = self.infer(image)
        if persons:
            print "P: {} C: {}".format(persons, confidences)

    def reconfig(self, config, level):
        if config.train:
            config.train = False
        return config

if __name__ == '__main__':
    rospy.init_node("face_recognizer")
    recognizer = FaceRecognizer()
    recognizer.prepare()
    Server(FaceRecognitionConfig, recognizer.reconfig)
    rospy.Subscriber('/camera/image_raw', Image, recognizer.image_cb)
    rospy.spin()
