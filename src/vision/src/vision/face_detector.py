import dlib
import numpy as np
import cv2
import os

HR_MODELS = os.environ.get('HR_MODELS', os.path.expanduser('~/.hr/cache/models'))
ARCHIVE_DIR = os.path.expanduser('~/.hr/faces')
DLIB_FACEPREDICTOR = os.path.join(HR_MODELS,
                    'shape_predictor_68_face_landmarks.dat')

dlib_face_detector = dlib.get_frontal_face_detector()
dlib_face_pose_predictor = dlib.shape_predictor(DLIB_FACEPREDICTOR)

def dlib_detect_face(image, **kwargs):
    nparr = np.fromstring(image, np.uint8)
    image = cv2.imdecode(nparr, 1)
    response = {}
    faces = []
    if image is not None and image.any():
        bboxes = dlib_face_detector(image)
        for bbox in bboxes:
            faces.append((bbox.left(),bbox.top(),bbox.right(),bbox.bottom()))

        if kwargs.get('landmarks'):
            all_landmarks = []
            for bbox in bboxes:
                landmarks = []
                shape = dlib_face_pose_predictor(image, bbox)
                for i in range(shape.num_parts):
                    point = shape.part(i)
                    x = int(point.x)
                    y = int(point.y)
                    landmarks.append((x, y))
                all_landmarks.append(landmarks)
            response['landmarks'] = all_landmarks
        response['faces'] = faces
        return response

detect_face = dlib_detect_face
