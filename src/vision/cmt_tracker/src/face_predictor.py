#!/usr/bin/env python2

import rospy
import logging
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from cmt_tracker_msgs.msg import Trackers,Tracker,Objects
from cmt_tracker_msgs.srv import TrackerNames
from cmt_tracker_msgs.cfg import RecogntionConfig

from dynamic_reconfigure.server import Server

from openface_wrapper import face_recognizer


class face_predictor:
  def __init__(self):
    rospy.init_node('face_recognizer', anonymous=True)
    self.openface_loc = rospy.get_param('openface')
    self.camera_topic = rospy.get_param('camera_topic')
    self.filtered_face_locations = rospy.get_param('filtered_face_locations')
    self.shape_predictor_file = rospy.get_param("shape_predictor")
    self.image_dir = rospy.get_param("image_locations")
    self.face_recognizer = face_recognizer(self.openface_loc, self.image_dir)
    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber(self.camera_topic, Image)
    self.cmt_sub = message_filters.Subscriber('tracker_results',Trackers)
    self.face_sub = message_filters.Subscriber(self.filtered_face_locations, Objects)
    self.temp_sub = message_filters.Subscriber('temporary_trackers',Trackers)
    ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cmt_sub,self.face_sub,self.temp_sub], 1,0.25)
    ts.registerCallback(self.callback)
    Server(RecogntionConfig, self.sample_callback)
    self.faces_cmt_overlap = {}
    self.logger = logging.getLogger('hr.cmt_tracker.face_recognizer_node')
    #This would hold the trackers that the system would hold.
    self.save_tracker_images = []
    #This query x results of this particular tracking instances.
    self.get_tracker_results = []

    self.cmt_state = {}

    self.states = ['results','save']
    self.overall_state = ''
    #So the logic goes; if the trainer is called when save_tracker_images has saved enough images.
    #And for get_tracker_results -> returns the

  def callback(self,data, cmt, face,temp):
    print('Overlap on face and tracker')
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ttp = cmt
    for val in temp.tracker_results:
        ttp.tracker_results.append(val)
    not_covered_faces,covered_faces = self.returnOverlapping(face,ttp)
    print('gets here')
    for face,cmt in covered_faces:
        tupl = []
        for pts in face.feature_point.points:
            x,y = pts.x, pts.y
            tupl.append((x,y))

        # First let's get the state of the face_recognzier.
        if not self.face_recognizer.get_state():
            self.overall_state ='save'
            print('overall state is save')
        self.faces_cmt_overlap[cmt.tracker_name.data] = self.faces_cmt_overlap.get(cmt.tracker_name.data,0) + 1
        print(self.faces_cmt_overlap[cmt.tracker_name.data])
        #State ONE-> Check for the results of a tracker.
        if self.faces_cmt_overlap[cmt.tracker_name.data] < self.sample_size and not cmt.recognized.data and (self.overall_state is not 'save'): # The first 10 images
            print('getting results')
            self.face_recognizer.results(cv_image,tupl,cmt.tracker_name.data)
        #State two Save faces
        elif self.faces_cmt_overlap[cmt.tracker_name.data] < (self.image_sample_size + self.sample_size) and not cmt.recognized.data:
            print('saving faces')
            self.face_recognizer.save_faces(cv_image,tupl,cmt.tracker_name.data,str(self.faces_cmt_overlap[cmt.tracker_name.data]))
        #TODO this shouldn't exist here at all.
        #State Three Train classifers.
        elif self.faces_cmt_overlap[cmt.tracker_name.data] == self.image_sample_size + self.sample_size:
            print('training process')
            self.face_recognizer.train_process(cmt.tracker_name.data)
        else:
            print('reaches new heights')
            #TODO one can use a reset here in the count to query results again. That depending on how to start saving
            pass

        #TODO how can we recall this.
        if self.faces_cmt_overlap[cmt.tracker_name.data] == self.sample_size and not cmt.recognized.data and (self.overall_state is not 'save'):
            print(self.face_recognizer.face_results_aggregator)
            max_index = max(self.face_recognizer.face_results_aggregator[cmt.tracker_name.data]['results'].iterkeys(),
                            key=(
                            lambda key: self.face_recognizer.face_results_aggregator[cmt.tracker_name.data]['results']))
            print(max_index)
            try:
                self.upt = rospy.ServiceProxy('recognition', TrackerNames)
                indication = self.upt(names=cmt.tracker_name.data, index=int(max_index[5:]))
                if not indication:
                    print("there was the same id in the id chamber.....")
            except rospy.ServiceException, e:
                print("Service call failed: %s" % e)
        #TODO occussional reinforcement by querying again and check results. That would be best if there is a feedback mechanism

  def sample_callback(self,config, level):
      self.image_sample_size = config.image_number
      self.sample_size = config.sample_size
      return config


  def returnOverlapping(self, face, cmt):
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
                list = [j,i]
                overlaped_faces.append(list)
                break
        if not overlap:
            not_covered_faces.append(j)
    return not_covered_faces, overlaped_faces



if __name__ == '__main__':
    ic = face_predictor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        logging.warn("Shutting down")


