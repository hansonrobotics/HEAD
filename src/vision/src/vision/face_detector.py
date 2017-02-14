import dlib
import numpy as np
import cv2

dlib_face_detector = dlib.get_frontal_face_detector()

def dlib_detect_face(image, **kwargs):
    nparr = np.fromstring(image, np.uint8)
    image = cv2.imdecode(nparr, 1)
    if image is not None and image.any():
        detected_faces = dlib_face_detector(image)
        return detected_faces

detect_face = dlib_detect_face
