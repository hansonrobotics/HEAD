; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-srv)


;//! \htmlinclude TrackerNames-request.msg.html

(cl:defclass <TrackerNames-request> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type cl:string
    :initform "")
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0))
)

(cl:defclass TrackerNames-request (<TrackerNames-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackerNames-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackerNames-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<TrackerNames-request> is deprecated: use cmt_tracker_msgs-srv:TrackerNames-request instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <TrackerNames-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:names-val is deprecated.  Use cmt_tracker_msgs-srv:names instead.")
  (names m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <TrackerNames-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:index-val is deprecated.  Use cmt_tracker_msgs-srv:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackerNames-request>) ostream)
  "Serializes a message object of type '<TrackerNames-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'names))
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackerNames-request>) istream)
  "Deserializes a message object of type '<TrackerNames-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'names) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'names) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackerNames-request>)))
  "Returns string type for a service object of type '<TrackerNames-request>"
  "cmt_tracker_msgs/TrackerNamesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackerNames-request)))
  "Returns string type for a service object of type 'TrackerNames-request"
  "cmt_tracker_msgs/TrackerNamesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackerNames-request>)))
  "Returns md5sum for a message object of type '<TrackerNames-request>"
  "8538b023dbdc0b64738453dc45bb1b1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackerNames-request)))
  "Returns md5sum for a message object of type 'TrackerNames-request"
  "8538b023dbdc0b64738453dc45bb1b1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackerNames-request>)))
  "Returns full string definition for message of type '<TrackerNames-request>"
  (cl:format cl:nil "string names~%int32 index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackerNames-request)))
  "Returns full string definition for message of type 'TrackerNames-request"
  (cl:format cl:nil "string names~%int32 index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackerNames-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'names))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackerNames-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackerNames-request
    (cl:cons ':names (names msg))
    (cl:cons ':index (index msg))
))
;//! \htmlinclude TrackerNames-response.msg.html

(cl:defclass <TrackerNames-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrackerNames-response (<TrackerNames-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackerNames-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackerNames-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<TrackerNames-response> is deprecated: use cmt_tracker_msgs-srv:TrackerNames-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackerNames-response>) ostream)
  "Serializes a message object of type '<TrackerNames-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackerNames-response>) istream)
  "Deserializes a message object of type '<TrackerNames-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackerNames-response>)))
  "Returns string type for a service object of type '<TrackerNames-response>"
  "cmt_tracker_msgs/TrackerNamesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackerNames-response)))
  "Returns string type for a service object of type 'TrackerNames-response"
  "cmt_tracker_msgs/TrackerNamesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackerNames-response>)))
  "Returns md5sum for a message object of type '<TrackerNames-response>"
  "8538b023dbdc0b64738453dc45bb1b1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackerNames-response)))
  "Returns md5sum for a message object of type 'TrackerNames-response"
  "8538b023dbdc0b64738453dc45bb1b1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackerNames-response>)))
  "Returns full string definition for message of type '<TrackerNames-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackerNames-response)))
  "Returns full string definition for message of type 'TrackerNames-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackerNames-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackerNames-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackerNames-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrackerNames)))
  'TrackerNames-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrackerNames)))
  'TrackerNames-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackerNames)))
  "Returns string type for a service object of type '<TrackerNames>"
  "cmt_tracker_msgs/TrackerNames")