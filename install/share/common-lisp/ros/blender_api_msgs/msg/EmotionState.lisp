; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude EmotionState.msg.html

(cl:defclass <EmotionState> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (magnitude
    :reader magnitude
    :initarg :magnitude
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:real
    :initform 0))
)

(cl:defclass EmotionState (<EmotionState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<EmotionState> is deprecated: use blender_api_msgs-msg:EmotionState instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <EmotionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:name-val is deprecated.  Use blender_api_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'magnitude-val :lambda-list '(m))
(cl:defmethod magnitude-val ((m <EmotionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:magnitude-val is deprecated.  Use blender_api_msgs-msg:magnitude instead.")
  (magnitude m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <EmotionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:duration-val is deprecated.  Use blender_api_msgs-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionState>) ostream)
  "Serializes a message object of type '<EmotionState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'magnitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'duration)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'duration) (cl:floor (cl:slot-value msg 'duration)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionState>) istream)
  "Deserializes a message object of type '<EmotionState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'magnitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionState>)))
  "Returns string type for a message object of type '<EmotionState>"
  "blender_api_msgs/EmotionState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionState)))
  "Returns string type for a message object of type 'EmotionState"
  "blender_api_msgs/EmotionState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionState>)))
  "Returns md5sum for a message object of type '<EmotionState>"
  "32bf35d00d56bb6221c497744c8f00dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionState)))
  "Returns md5sum for a message object of type 'EmotionState"
  "32bf35d00d56bb6221c497744c8f00dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionState>)))
  "Returns full string definition for message of type '<EmotionState>"
  (cl:format cl:nil "string name~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionState)))
  "Returns full string definition for message of type 'EmotionState"
  (cl:format cl:nil "string name~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionState>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionState
    (cl:cons ':name (name msg))
    (cl:cons ':magnitude (magnitude msg))
    (cl:cons ':duration (duration msg))
))
