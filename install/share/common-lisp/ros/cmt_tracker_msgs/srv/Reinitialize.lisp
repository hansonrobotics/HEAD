; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-srv)


;//! \htmlinclude Reinitialize-request.msg.html

(cl:defclass <Reinitialize-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass Reinitialize-request (<Reinitialize-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reinitialize-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reinitialize-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Reinitialize-request> is deprecated: use cmt_tracker_msgs-srv:Reinitialize-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Reinitialize-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:object-val is deprecated.  Use cmt_tracker_msgs-srv:object instead.")
  (object m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Reinitialize-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:name-val is deprecated.  Use cmt_tracker_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reinitialize-request>) ostream)
  "Serializes a message object of type '<Reinitialize-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reinitialize-request>) istream)
  "Deserializes a message object of type '<Reinitialize-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reinitialize-request>)))
  "Returns string type for a service object of type '<Reinitialize-request>"
  "cmt_tracker_msgs/ReinitializeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reinitialize-request)))
  "Returns string type for a service object of type 'Reinitialize-request"
  "cmt_tracker_msgs/ReinitializeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reinitialize-request>)))
  "Returns md5sum for a message object of type '<Reinitialize-request>"
  "0392f3ba1598a6396d8981fc037597b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reinitialize-request)))
  "Returns md5sum for a message object of type 'Reinitialize-request"
  "0392f3ba1598a6396d8981fc037597b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reinitialize-request>)))
  "Returns full string definition for message of type '<Reinitialize-request>"
  (cl:format cl:nil "sensor_msgs/RegionOfInterest object~%string name~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reinitialize-request)))
  "Returns full string definition for message of type 'Reinitialize-request"
  (cl:format cl:nil "sensor_msgs/RegionOfInterest object~%string name~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reinitialize-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reinitialize-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Reinitialize-request
    (cl:cons ':object (object msg))
    (cl:cons ':name (name msg))
))
;//! \htmlinclude Reinitialize-response.msg.html

(cl:defclass <Reinitialize-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Reinitialize-response (<Reinitialize-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reinitialize-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reinitialize-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Reinitialize-response> is deprecated: use cmt_tracker_msgs-srv:Reinitialize-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reinitialize-response>) ostream)
  "Serializes a message object of type '<Reinitialize-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reinitialize-response>) istream)
  "Deserializes a message object of type '<Reinitialize-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reinitialize-response>)))
  "Returns string type for a service object of type '<Reinitialize-response>"
  "cmt_tracker_msgs/ReinitializeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reinitialize-response)))
  "Returns string type for a service object of type 'Reinitialize-response"
  "cmt_tracker_msgs/ReinitializeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reinitialize-response>)))
  "Returns md5sum for a message object of type '<Reinitialize-response>"
  "0392f3ba1598a6396d8981fc037597b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reinitialize-response)))
  "Returns md5sum for a message object of type 'Reinitialize-response"
  "0392f3ba1598a6396d8981fc037597b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reinitialize-response>)))
  "Returns full string definition for message of type '<Reinitialize-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reinitialize-response)))
  "Returns full string definition for message of type 'Reinitialize-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reinitialize-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reinitialize-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Reinitialize-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Reinitialize)))
  'Reinitialize-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Reinitialize)))
  'Reinitialize-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reinitialize)))
  "Returns string type for a service object of type '<Reinitialize>"
  "cmt_tracker_msgs/Reinitialize")