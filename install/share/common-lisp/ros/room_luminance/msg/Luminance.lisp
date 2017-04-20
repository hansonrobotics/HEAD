; Auto-generated. Do not edit!


(cl:in-package room_luminance-msg)


;//! \htmlinclude Luminance.msg.html

(cl:defclass <Luminance> (roslisp-msg-protocol:ros-message)
  ((covered
    :reader covered
    :initarg :covered
    :type cl:fixnum
    :initform 0)
   (sudden_change
    :reader sudden_change
    :initarg :sudden_change
    :type cl:fixnum
    :initform 0)
   (room_light
    :reader room_light
    :initarg :room_light
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (perc_covered
    :reader perc_covered
    :initarg :perc_covered
    :type cl:float
    :initform 0.0))
)

(cl:defclass Luminance (<Luminance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Luminance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Luminance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name room_luminance-msg:<Luminance> is deprecated: use room_luminance-msg:Luminance instead.")))

(cl:ensure-generic-function 'covered-val :lambda-list '(m))
(cl:defmethod covered-val ((m <Luminance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader room_luminance-msg:covered-val is deprecated.  Use room_luminance-msg:covered instead.")
  (covered m))

(cl:ensure-generic-function 'sudden_change-val :lambda-list '(m))
(cl:defmethod sudden_change-val ((m <Luminance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader room_luminance-msg:sudden_change-val is deprecated.  Use room_luminance-msg:sudden_change instead.")
  (sudden_change m))

(cl:ensure-generic-function 'room_light-val :lambda-list '(m))
(cl:defmethod room_light-val ((m <Luminance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader room_luminance-msg:room_light-val is deprecated.  Use room_luminance-msg:room_light instead.")
  (room_light m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Luminance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader room_luminance-msg:value-val is deprecated.  Use room_luminance-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'perc_covered-val :lambda-list '(m))
(cl:defmethod perc_covered-val ((m <Luminance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader room_luminance-msg:perc_covered-val is deprecated.  Use room_luminance-msg:perc_covered instead.")
  (perc_covered m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Luminance>) ostream)
  "Serializes a message object of type '<Luminance>"
  (cl:let* ((signed (cl:slot-value msg 'covered)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sudden_change)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'room_light))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'room_light))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'perc_covered))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Luminance>) istream)
  "Deserializes a message object of type '<Luminance>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'covered) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sudden_change) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'room_light) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'room_light) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'perc_covered) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Luminance>)))
  "Returns string type for a message object of type '<Luminance>"
  "room_luminance/Luminance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Luminance)))
  "Returns string type for a message object of type 'Luminance"
  "room_luminance/Luminance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Luminance>)))
  "Returns md5sum for a message object of type '<Luminance>"
  "808071152779e2b1174c938d5216ce43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Luminance)))
  "Returns md5sum for a message object of type 'Luminance"
  "808071152779e2b1174c938d5216ce43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Luminance>)))
  "Returns full string definition for message of type '<Luminance>"
  (cl:format cl:nil "int8 covered~%int8 sudden_change~%string room_light~%float32 value~%float32 perc_covered~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Luminance)))
  "Returns full string definition for message of type 'Luminance"
  (cl:format cl:nil "int8 covered~%int8 sudden_change~%string room_light~%float32 value~%float32 perc_covered~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Luminance>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'room_light))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Luminance>))
  "Converts a ROS message object to a list"
  (cl:list 'Luminance
    (cl:cons ':covered (covered msg))
    (cl:cons ':sudden_change (sudden_change msg))
    (cl:cons ':room_light (room_light msg))
    (cl:cons ':value (value msg))
    (cl:cons ':perc_covered (perc_covered msg))
))
