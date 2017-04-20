; Auto-generated. Do not edit!


(cl:in-package pi_face_tracker-srv)


;//! \htmlinclude SetROI-request.msg.html

(cl:defclass <SetROI-request> (roslisp-msg-protocol:ros-message)
  ((roi
    :reader roi
    :initarg :roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest)))
)

(cl:defclass SetROI-request (<SetROI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetROI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetROI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-srv:<SetROI-request> is deprecated: use pi_face_tracker-srv:SetROI-request instead.")))

(cl:ensure-generic-function 'roi-val :lambda-list '(m))
(cl:defmethod roi-val ((m <SetROI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-srv:roi-val is deprecated.  Use pi_face_tracker-srv:roi instead.")
  (roi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetROI-request>) ostream)
  "Serializes a message object of type '<SetROI-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'roi) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetROI-request>) istream)
  "Deserializes a message object of type '<SetROI-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'roi) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetROI-request>)))
  "Returns string type for a service object of type '<SetROI-request>"
  "pi_face_tracker/SetROIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI-request)))
  "Returns string type for a service object of type 'SetROI-request"
  "pi_face_tracker/SetROIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetROI-request>)))
  "Returns md5sum for a message object of type '<SetROI-request>"
  "9586986cb2458d45fdbda6087252affd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetROI-request)))
  "Returns md5sum for a message object of type 'SetROI-request"
  "9586986cb2458d45fdbda6087252affd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetROI-request>)))
  "Returns full string definition for message of type '<SetROI-request>"
  (cl:format cl:nil "sensor_msgs/RegionOfInterest roi~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetROI-request)))
  "Returns full string definition for message of type 'SetROI-request"
  (cl:format cl:nil "sensor_msgs/RegionOfInterest roi~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetROI-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'roi))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetROI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetROI-request
    (cl:cons ':roi (roi msg))
))
;//! \htmlinclude SetROI-response.msg.html

(cl:defclass <SetROI-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetROI-response (<SetROI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetROI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetROI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-srv:<SetROI-response> is deprecated: use pi_face_tracker-srv:SetROI-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetROI-response>) ostream)
  "Serializes a message object of type '<SetROI-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetROI-response>) istream)
  "Deserializes a message object of type '<SetROI-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetROI-response>)))
  "Returns string type for a service object of type '<SetROI-response>"
  "pi_face_tracker/SetROIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI-response)))
  "Returns string type for a service object of type 'SetROI-response"
  "pi_face_tracker/SetROIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetROI-response>)))
  "Returns md5sum for a message object of type '<SetROI-response>"
  "9586986cb2458d45fdbda6087252affd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetROI-response)))
  "Returns md5sum for a message object of type 'SetROI-response"
  "9586986cb2458d45fdbda6087252affd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetROI-response>)))
  "Returns full string definition for message of type '<SetROI-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetROI-response)))
  "Returns full string definition for message of type 'SetROI-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetROI-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetROI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetROI-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetROI)))
  'SetROI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetROI)))
  'SetROI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI)))
  "Returns string type for a service object of type '<SetROI>"
  "pi_face_tracker/SetROI")