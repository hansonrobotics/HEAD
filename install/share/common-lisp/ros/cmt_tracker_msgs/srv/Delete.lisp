; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-srv)


;//! \htmlinclude Delete-request.msg.html

(cl:defclass <Delete-request> (roslisp-msg-protocol:ros-message)
  ((delete_trackers
    :reader delete_trackers
    :initarg :delete_trackers
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Delete-request (<Delete-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Delete-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Delete-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Delete-request> is deprecated: use cmt_tracker_msgs-srv:Delete-request instead.")))

(cl:ensure-generic-function 'delete_trackers-val :lambda-list '(m))
(cl:defmethod delete_trackers-val ((m <Delete-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-srv:delete_trackers-val is deprecated.  Use cmt_tracker_msgs-srv:delete_trackers instead.")
  (delete_trackers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Delete-request>) ostream)
  "Serializes a message object of type '<Delete-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'delete_trackers))))
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
   (cl:slot-value msg 'delete_trackers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Delete-request>) istream)
  "Deserializes a message object of type '<Delete-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'delete_trackers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'delete_trackers)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Delete-request>)))
  "Returns string type for a service object of type '<Delete-request>"
  "cmt_tracker_msgs/DeleteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete-request)))
  "Returns string type for a service object of type 'Delete-request"
  "cmt_tracker_msgs/DeleteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Delete-request>)))
  "Returns md5sum for a message object of type '<Delete-request>"
  "76b88b33c93b522263cd2661b6ecc420")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Delete-request)))
  "Returns md5sum for a message object of type 'Delete-request"
  "76b88b33c93b522263cd2661b6ecc420")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Delete-request>)))
  "Returns full string definition for message of type '<Delete-request>"
  (cl:format cl:nil "string[] delete_trackers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Delete-request)))
  "Returns full string definition for message of type 'Delete-request"
  (cl:format cl:nil "string[] delete_trackers~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Delete-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'delete_trackers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Delete-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Delete-request
    (cl:cons ':delete_trackers (delete_trackers msg))
))
;//! \htmlinclude Delete-response.msg.html

(cl:defclass <Delete-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Delete-response (<Delete-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Delete-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Delete-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-srv:<Delete-response> is deprecated: use cmt_tracker_msgs-srv:Delete-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Delete-response>) ostream)
  "Serializes a message object of type '<Delete-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Delete-response>) istream)
  "Deserializes a message object of type '<Delete-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Delete-response>)))
  "Returns string type for a service object of type '<Delete-response>"
  "cmt_tracker_msgs/DeleteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete-response)))
  "Returns string type for a service object of type 'Delete-response"
  "cmt_tracker_msgs/DeleteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Delete-response>)))
  "Returns md5sum for a message object of type '<Delete-response>"
  "76b88b33c93b522263cd2661b6ecc420")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Delete-response)))
  "Returns md5sum for a message object of type 'Delete-response"
  "76b88b33c93b522263cd2661b6ecc420")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Delete-response>)))
  "Returns full string definition for message of type '<Delete-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Delete-response)))
  "Returns full string definition for message of type 'Delete-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Delete-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Delete-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Delete-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Delete)))
  'Delete-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Delete)))
  'Delete-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delete)))
  "Returns string type for a service object of type '<Delete>"
  "cmt_tracker_msgs/Delete")