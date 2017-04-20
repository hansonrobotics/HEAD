; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-srv)


;//! \htmlinclude Update-request.msg.html

(cl:defclass <Update-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Update-request (<Update-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Update-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Update-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Update-request> is deprecated: use cmt_tracker_msgs-srv:Update-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Update-request>) ostream)
  "Serializes a message object of type '<Update-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Update-request>) istream)
  "Deserializes a message object of type '<Update-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Update-request>)))
  "Returns string type for a service object of type '<Update-request>"
  "cmt_tracker_msgs/UpdateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Update-request)))
  "Returns string type for a service object of type 'Update-request"
  "cmt_tracker_msgs/UpdateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Update-request>)))
  "Returns md5sum for a message object of type '<Update-request>"
  "0f1916e89211fc428edc7faefc5b53b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Update-request)))
  "Returns md5sum for a message object of type 'Update-request"
  "0f1916e89211fc428edc7faefc5b53b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Update-request>)))
  "Returns full string definition for message of type '<Update-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Update-request)))
  "Returns full string definition for message of type 'Update-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Update-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Update-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Update-request
))
;//! \htmlinclude Update-response.msg.html

(cl:defclass <Update-response> (roslisp-msg-protocol:ros-message)
  ((update
    :reader update
    :initarg :update
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass Update-response (<Update-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Update-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Update-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Update-response> is deprecated: use cmt_tracker_msgs-srv:Update-response instead.")))

(cl:ensure-generic-function 'update-val :lambda-list '(m))
(cl:defmethod update-val ((m <Update-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:update-val is deprecated.  Use cmt_tracker_msgs-srv:update instead.")
  (update m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Update-response>) ostream)
  "Serializes a message object of type '<Update-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'update) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Update-response>) istream)
  "Deserializes a message object of type '<Update-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'update) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Update-response>)))
  "Returns string type for a service object of type '<Update-response>"
  "cmt_tracker_msgs/UpdateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Update-response)))
  "Returns string type for a service object of type 'Update-response"
  "cmt_tracker_msgs/UpdateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Update-response>)))
  "Returns md5sum for a message object of type '<Update-response>"
  "0f1916e89211fc428edc7faefc5b53b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Update-response)))
  "Returns md5sum for a message object of type 'Update-response"
  "0f1916e89211fc428edc7faefc5b53b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Update-response>)))
  "Returns full string definition for message of type '<Update-response>"
  (cl:format cl:nil "std_msgs/Bool update~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Update-response)))
  "Returns full string definition for message of type 'Update-response"
  (cl:format cl:nil "std_msgs/Bool update~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Update-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'update))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Update-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Update-response
    (cl:cons ':update (update msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Update)))
  'Update-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Update)))
  'Update-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Update)))
  "Returns string type for a service object of type '<Update>"
  "cmt_tracker_msgs/Update")