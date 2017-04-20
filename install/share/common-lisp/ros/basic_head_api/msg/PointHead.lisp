; Auto-generated. Do not edit!


(cl:in-package basic_head_api-msg)


;//! \htmlinclude PointHead.msg.html

(cl:defclass <PointHead> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0))
)

(cl:defclass PointHead (<PointHead>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointHead>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointHead)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-msg:<PointHead> is deprecated: use basic_head_api-msg:PointHead instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <PointHead>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:yaw-val is deprecated.  Use basic_head_api-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <PointHead>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:pitch-val is deprecated.  Use basic_head_api-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <PointHead>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:roll-val is deprecated.  Use basic_head_api-msg:roll instead.")
  (roll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointHead>) ostream)
  "Serializes a message object of type '<PointHead>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointHead>) istream)
  "Deserializes a message object of type '<PointHead>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointHead>)))
  "Returns string type for a message object of type '<PointHead>"
  "basic_head_api/PointHead")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointHead)))
  "Returns string type for a message object of type 'PointHead"
  "basic_head_api/PointHead")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointHead>)))
  "Returns md5sum for a message object of type '<PointHead>"
  "5b83d17121a77a6856144c74d9af68ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointHead)))
  "Returns md5sum for a message object of type 'PointHead"
  "5b83d17121a77a6856144c74d9af68ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointHead>)))
  "Returns full string definition for message of type '<PointHead>"
  (cl:format cl:nil "float32 yaw~%float32 pitch~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointHead)))
  "Returns full string definition for message of type 'PointHead"
  (cl:format cl:nil "float32 yaw~%float32 pitch~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointHead>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointHead>))
  "Converts a ROS message object to a list"
  (cl:list 'PointHead
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
))
