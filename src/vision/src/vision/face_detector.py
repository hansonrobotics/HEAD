import dlib
import numpy as np
import cv2

face_detector = dlib.get_frontal_face_detector()

def detect_face(image, max_results=4):
    nparr = np.fromstring(image, np.uint8)
    image = cv2.imdecode(nparr, 1)
    if image is not None and image.any():
        detected_faces = face_detector(image)
        return detected_faces
