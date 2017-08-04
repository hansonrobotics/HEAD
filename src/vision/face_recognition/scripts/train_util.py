import os
import cv2
import dlib
import openface
import random
import pandas as pd
import logging
import pickle
from openface.data import iterImgs
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC

HR_MODELS = os.environ.get('HR_MODELS', os.path.expanduser('~/.hr/cache/models'))
DLIB_FACEPREDICTOR = os.path.join(HR_MODELS,
                    'shape_predictor_68_face_landmarks.dat')
NETWORK_MODEL = os.path.join(HR_MODELS, 'nn4.small2.v1.t7')
CLASSIFIER_DIR = os.path.join(HR_MODELS, 'classifier')

logger = logging.getLogger('hr.ros_face_recognition.train_util')

class TrainUtil(object):

    def __init__(self, train_dir, aligned_dir, classifier_dir):
        self.train_dir = train_dir
        self.aligned_dir = aligned_dir
        self.classifier_dir = classifier_dir

        self.align = openface.AlignDlib(DLIB_FACEPREDICTOR)
        self.landmarkIndices = openface.AlignDlib.OUTER_EYES_AND_NOSE
        self.imgDim = 96
        self.net = openface.TorchNeuralNet(NETWORK_MODEL, self.imgDim)
        for d in [self.train_dir, self.aligned_dir, self.classifier_dir]:
            if not os.path.isdir(d):
                os.makedirs(d)

    def align_images(self):
        imgs = list(iterImgs(self.train_dir))
        # Shuffle so multiple versions can be run at once.
        random.shuffle(imgs)
        n_total = len(imgs)

        for i, imgObject in enumerate(imgs, 1):
            outDir = os.path.join(self.aligned_dir, imgObject.cls)
            if not os.path.isdir(outDir):
                os.makedirs(outDir)
            outputPrefix = os.path.join(outDir, imgObject.name)
            imgName = outputPrefix + ".png"

            if not os.path.isfile(imgName):
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
                    logger.info("Write image {}".format(imgName))
                else:
                    os.remove(imgObject.path)
                    logger.warn("No face was detected in {}. Removed.".format(imgObject.path))
            logger.info("{}/{}".format(i, n_total))

    def gen_data(self):
        face_reps = []
        labels = []
        reps_fname = "{}/reps.csv".format(self.classifier_dir)
        label_fname = "{}/labels.csv".format(self.classifier_dir)

        imgs = list(iterImgs(self.aligned_dir))
        n_total = len(imgs)
        for i, imgObject in enumerate(imgs, 1):
            reps = self.net.forward(imgObject.getRGB())
            face_reps.append(reps)
            labels.append((imgObject.cls, imgObject.name))
            logger.info("{}/{}".format(i, n_total))

        if face_reps and labels:
            pd.DataFrame(face_reps).to_csv(reps_fname, header=False, index=False)
            pd.DataFrame(labels).to_csv(label_fname, header=False, index=False)
            logger.info("Generated label file {}".format(label_fname))
            logger.info("Generated representation file {}".format(reps_fname))

    def train_model(self):
        label_fname = "{}/labels.csv".format(self.classifier_dir)
        reps_fname = "{}/reps.csv".format(self.classifier_dir)
        classifier_fname = "{}/classifier.pkl".format(self.classifier_dir)

        labels, embeddings = None, None
        if os.path.isfile(label_fname) and \
                    os.path.isfile(reps_fname):
            labels = pd.read_csv(label_fname, header=None)
            embeddings = pd.read_csv(reps_fname, header=None)

        if labels is None or embeddings is None:
            logger.error("No labels or representations are found")
            return

        labels_data = labels.as_matrix()[:,0].tolist()
        embeddings_data = embeddings.as_matrix()

        le = LabelEncoder().fit(labels_data)
        labelsNum = le.transform(labels_data)
        clf = SVC(C=1, kernel='linear', probability=True)
        try:
            logger.info("Start training")
            clf.fit(embeddings_data, labelsNum)
        except ValueError as ex:
            logger.error(ex)
            return

        with open(classifier_fname, 'w') as f:
            pickle.dump((le, clf), f)
        logger.info("Model saved to {}".format(classifier_fname))

if __name__ == '__main__':
    BASIC_FORMAT = "%(asctime)s:%(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(format=BASIC_FORMAT, level=logging.INFO)
    root_dir = 'lfw'
    train_dir = os.path.join(root_dir, 'training-images')
    aligned_dir = os.path.join(root_dir, 'aligned-images')
    classifier_dir = os.path.join(root_dir, 'classifier')
    util = TrainUtil(train_dir, aligned_dir, classifier_dir)
    util.align_images()
    util.gen_data()
    util.train_model()
