; Auto-generated. Do not edit!


(cl:in-package basic_head_api-msg)


;//! \htmlinclude PlayAnimation.msg.html

(cl:defclass <PlayAnimation> (roslisp-msg-protocol:ros-message)
  ((animation
    :reader animation
    :initarg :animation
    :type cl:string
    :initform "")
   (fps
    :reader fps
    :initarg :fps
    :type cl:integer
    :initform 0))
)

(cl:defclass PlayAnimation (<PlayAnimation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayAnimation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayAnimation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-msg:<PlayAnimation> is deprecated: use basic_head_api-msg:PlayAnimation instead.")))

(cl:ensure-generic-function 'animation-val :lambda-list '(m))
(cl:defmethod animation-val ((m <PlayAnimation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:animation-val is deprecated.  Use basic_head_api-msg:animation instead.")
  (animation m))

(cl:ensure-generic-function 'fps-val :lambda-list '(m))
(cl:defmethod fps-val ((m <PlayAnimation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:fps-val is deprecated.  Use basic_head_api-msg:fps instead.")
  (fps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayAnimation>) ostream)
  "Serializes a message object of type '<PlayAnimation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'animation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'animation))
  (cl:let* ((signed (cl:slot-value msg 'fps)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayAnimation>) istream)
  "Deserializes a message object of type '<PlayAnimation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'animation) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'animation) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fps) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayAnimation>)))
  "Returns string type for a message object of type '<PlayAnimation>"
  "basic_head_api/PlayAnimation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayAnimation)))
  "Returns string type for a message object of type 'PlayAnimation"
  "basic_head_api/PlayAnimation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayAnimation>)))
  "Returns md5sum for a message object of type '<PlayAnimation>"
  "c00c0ae8e42a687c424ac0f67577887a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayAnimation)))
  "Returns md5sum for a message object of type 'PlayAnimation"
  "c00c0ae8e42a687c424ac0f67577887a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayAnimation>)))
  "Returns full string definition for message of type '<PlayAnimation>"
  (cl:format cl:nil "string animation~%int64 fps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayAnimation)))
  "Returns full string definition for message of type 'PlayAnimation"
  (cl:format cl:nil "string animation~%int64 fps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayAnimation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'animation))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayAnimation>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayAnimation
    (cl:cons ':animation (animation msg))
    (cl:cons ':fps (fps msg))
))
