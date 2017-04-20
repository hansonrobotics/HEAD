; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-srv)


;//! \htmlinclude MergeNames-request.msg.html

(cl:defclass <MergeNames-request> (roslisp-msg-protocol:ros-message)
  ((merge_to
    :reader merge_to
    :initarg :merge_to
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (merge_from
    :reader merge_from
    :initarg :merge_from
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass MergeNames-request (<MergeNames-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MergeNames-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MergeNames-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<MergeNames-request> is deprecated: use cmt_tracker_msgs-srv:MergeNames-request instead.")))

(cl:ensure-generic-function 'merge_to-val :lambda-list '(m))
(cl:defmethod merge_to-val ((m <MergeNames-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:merge_to-val is deprecated.  Use cmt_tracker_msgs-srv:merge_to instead.")
  (merge_to m))

(cl:ensure-generic-function 'merge_from-val :lambda-list '(m))
(cl:defmethod merge_from-val ((m <MergeNames-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:merge_from-val is deprecated.  Use cmt_tracker_msgs-srv:merge_from instead.")
  (merge_from m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MergeNames-request>) ostream)
  "Serializes a message object of type '<MergeNames-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'merge_to))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'merge_to))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'merge_from))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'merge_from))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MergeNames-request>) istream)
  "Deserializes a message object of type '<MergeNames-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'merge_to) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'merge_to)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'merge_from) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'merge_from)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MergeNames-request>)))
  "Returns string type for a service object of type '<MergeNames-request>"
  "cmt_tracker_msgs/MergeNamesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MergeNames-request)))
  "Returns string type for a service object of type 'MergeNames-request"
  "cmt_tracker_msgs/MergeNamesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MergeNames-request>)))
  "Returns md5sum for a message object of type '<MergeNames-request>"
  "856e68983edeb33a1b49afa6ff24e5ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MergeNames-request)))
  "Returns md5sum for a message object of type 'MergeNames-request"
  "856e68983edeb33a1b49afa6ff24e5ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MergeNames-request>)))
  "Returns full string definition for message of type '<MergeNames-request>"
  (cl:format cl:nil "string[] merge_to~%string[] merge_from~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MergeNames-request)))
  "Returns full string definition for message of type 'MergeNames-request"
  (cl:format cl:nil "string[] merge_to~%string[] merge_from~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MergeNames-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'merge_to) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'merge_from) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MergeNames-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MergeNames-request
    (cl:cons ':merge_to (merge_to msg))
    (cl:cons ':merge_from (merge_from msg))
))
;//! \htmlinclude MergeNames-response.msg.html

(cl:defclass <MergeNames-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MergeNames-response (<MergeNames-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MergeNames-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MergeNames-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<MergeNames-response> is deprecated: use cmt_tracker_msgs-srv:MergeNames-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MergeNames-response>) ostream)
  "Serializes a message object of type '<MergeNames-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MergeNames-response>) istream)
  "Deserializes a message object of type '<MergeNames-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MergeNames-response>)))
  "Returns string type for a service object of type '<MergeNames-response>"
  "cmt_tracker_msgs/MergeNamesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MergeNames-response)))
  "Returns string type for a service object of type 'MergeNames-response"
  "cmt_tracker_msgs/MergeNamesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MergeNames-response>)))
  "Returns md5sum for a message object of type '<MergeNames-response>"
  "856e68983edeb33a1b49afa6ff24e5ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MergeNames-response)))
  "Returns md5sum for a message object of type 'MergeNames-response"
  "856e68983edeb33a1b49afa6ff24e5ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MergeNames-response>)))
  "Returns full string definition for message of type '<MergeNames-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MergeNames-response)))
  "Returns full string definition for message of type 'MergeNames-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MergeNames-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MergeNames-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MergeNames-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MergeNames)))
  'MergeNames-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MergeNames)))
  'MergeNames-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MergeNames)))
  "Returns string type for a service object of type '<MergeNames>"
  "cmt_tracker_msgs/MergeNames")