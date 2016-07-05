#!/usr/bin/env python2

import sys
import subprocess
import openface
import shutil
import os
import rospkg
import pickle
import pandas as pd
import openface
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from operator import itemgetter
from collections import defaultdict
import shutil
import cv2
import logging
import numpy as np

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
    '''
    Openface related tools for face classification.
    '''
    def __init__(self,openface_loc, image_dir):
        self.net = openface.TorchNeuralNet(openface_loc + '/models/openface/nn4.small2.v1.t7', imgDim=96,
                                           cuda=False)
        self.image_dir_face_imgs = image_dir + "/faces"
        self.image_dir_face_temp = image_dir + "/tmp"
        self.feature_dir = image_dir + "/feature"
        # Get Environmental variable for the file during start up.
        self.batch_represent = openface_loc + '/batch-represent/main.lua'

        self.inner_eyes_and_bottom_lip = [38, 42, 57]
        # This is one that is used for trained model nn4.small2
        self.outer_eyes_and_nose = [36, 45, 33]

        if (not os.path.exists(image_dir)):
            os.makedirs(image_dir)

        if (not os.path.exists(self.image_dir_face_imgs)):
            os.makedirs(self.image_dir_face_imgs)
        if (not os.path.exists(self.feature_dir)):
            os.makedirs(self.feature_dir)
        if (not os.path.exists(self.image_dir_face_temp)):
            os.makedirs(self.image_dir_face_temp)

        # Delete all the items in temp folder so as not to consume space. May be when it's brought down we can delete them. or in destructor without sigterm.
        for the_file in os.listdir(self.image_dir_face_temp):
            file_path = os.path.join(self.image_dir_face_temp, the_file)
            shutil.rmtree(file_path)
        self.logger = logging.getLogger('hr.cmt_tracker.face_reinforcer_node')
        self.face_results_aggregator = {}

        if os.path.exists("{}/classifier.pkl".format(self.feature_dir)):
            pass
        else:
            pass

    def train(self):
        self.logger.info('Training Dataset')
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
        self.logger.info('Finished Training')

    def infer(self, net_forwarded_img):
        self.logger.info('Inferring Result')
        if os.path.exists("{}/classifier.pkl".format(self.feature_dir)):
            with open("{}/classifier.pkl".format(self.feature_dir), 'r') as f:
                (le, svm) = pickle.load(f)
            prediction = svm.predict_proba(net_forwarded_img)[0]
            maxI = np.argmax(prediction)
            person = le.inverse_transform(maxI)
            confidence = prediction[maxI]
            self.logger.info('Finished Inferring')
            return (person, confidence)


    def align(self, img, landmarks, net=False):
        npLandmarks = np.float32(landmarks)
        npLandmarksIndices = np.array(self.outer_eyes_and_nose)
        H = cv2.getAffineTransform(npLandmarks[npLandmarksIndices],
                                   96 * MINMAX_TEMPLATE[npLandmarksIndices])
        thumbnail = cv2.warpAffine(img, H, (96, 96))
        if (net):
            thumbnail = self.net.forward(thumbnail).reshape(1, -1)
            return thumbnail
        return thumbnail


    def save_faces(self, cv_image, tupl, loc_save,postfix):
        self.logger.info("saving faces in %s", self.image_dir_face_temp + "/" + loc_save)
        if (not os.path.exists(self.image_dir_face_temp + "/" + loc_save)):
            os.makedirs(self.image_dir_face_temp + "/" + loc_save)
        img_aligned = self.align(cv_image, tupl)
        # Here let's create a method that it generated a name for itself.
        cv2.imwrite(self.image_dir_face_temp + "/" +loc_save + "/" + postfix + ".jpg", img_aligned)
        self.logger.info("saved faces %s",loc_save + "_" + postfix + ".jpg")

    def train_process(self, name):
        self.logger.info('starts training')
        file_loc = self.image_dir_face_temp + "/" + name
        if (os.path.exists(file_loc)):
            subprocess.call(['mv', self.image_dir_face_temp + "/" + name,
                         self.image_dir_face_imgs + "/" + name])
        # TO avoid calling training if the faces directory doesn't have faces at least two faces.
        if len(list(os.walk(self.image_dir_face_imgs).next()[1])) > 1:
            self.train_dataset()
        self.logger.info('finsihes training')

    def train_dataset(self):
        self.logger.info('Creating Representation')
        # As at this point the dataset has changed thusly we have to delete the cache.
        if (os.path.exists(self.image_dir_face_imgs + "/" + 'cache.t7')):
            subprocess.call(['rm', self.image_dir_face_imgs + "/" + 'cache.t7'])
            # Here call the batch represent file which is a lua function in a process of it's own.
        subprocess.call([self.batch_represent, '-outDir', self.feature_dir, '-data', self.image_dir_face_imgs])
        # here the number of images in the classifier must be greater than 1
        self.logger.info('Reset results to Zero')
        self.logger.info('Training on Representation')
        self.face_results_aggregator = {}
        self.train()

    def results(self, cv_image, tupl, name, threshold=0.85):
        self.logger.info('reaches results')
        # TODO Even take out this existence
        img_aligned = self.align(cv_image, tupl, True)
        result = self.infer(img_aligned)
        self.face_results_aggregator[name] = self.face_results_aggregator.get(name, {})
        self.face_results_aggregator[name]['results'] = self.face_results_aggregator[name].get(
            'results', {})
        self.face_results_aggregator[name]['results'][result[0]] = \
            self.face_results_aggregator[name]['results'].get(result[0], 0)

        self.face_results_aggregator[name] = self.face_results_aggregator.get(name, {})
        self.face_results_aggregator[name]['results'] = self.face_results_aggregator[name].get(
            'results', {})
        self.face_results_aggregator[name]['results'][result[0]] = \
            self.face_results_aggregator[name]['results'].get(result[0], 0)

        if result[1] > threshold:  ##TODO Dynamic reconfigurable name; if's below this threshold then stop.
            self.face_results_aggregator[name]['results'][result[0]] = \
                self.face_results_aggregator[name]['results'][result[0]] + 1
        else:
            self.face_results_aggregator[name]['results'][result[0]] = \
                self.face_results_aggregator[name]['results'][result[0]] + 0

        self.logger.info('finishes result')

    def immediate_results(self, cv_image, tupl):
        img_aligned = self.align(cv_image, tupl, True)
        result = self.infer(img_aligned)
        return result

    def get_state(self):
        '''
        @return: The existance of the classifier.pkl
        '''
        if os.path.exists("{}/classifier.pkl".format(self.feature_dir)):
            return True
        else:
            return False
    def clear_results(self):
        '''
        Clears the face_results_aggregator from all the results.
        @return:
        '''
        self.face_results_aggregator = {}

