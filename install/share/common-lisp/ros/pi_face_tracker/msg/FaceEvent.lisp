; Auto-generated. Do not edit!


(cl:in-package pi_face_tracker-msg)


;//! \htmlinclude FaceEvent.msg.html

(cl:defclass <FaceEvent> (roslisp-msg-protocol:ros-message)
  ((face_event
    :reader face_event
    :initarg :face_event
    :type cl:string
    :initform "")
   (face_id
    :reader face_id
    :initarg :face_id
    :type cl:integer
    :initform 0)
   (recognized_id
    :reader recognized_id
    :initarg :recognized_id
    :type cl:string
    :initform ""))
)

(cl:defclass FaceEvent (<FaceEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-msg:<FaceEvent> is deprecated: use pi_face_tracker-msg:FaceEvent instead.")))

(cl:ensure-generic-function 'face_event-val :lambda-list '(m))
(cl:defmethod face_event-val ((m <FaceEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-msg:face_event-val is deprecated.  Use pi_face_tracker-msg:face_event instead.")
  (face_event m))

(cl:ensure-generic-function 'face_id-val :lambda-list '(m))
(cl:defmethod face_id-val ((m <FaceEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-msg:face_id-val is deprecated.  Use pi_face_tracker-msg:face_id instead.")
  (face_id m))

(cl:ensure-generic-function 'recognized_id-val :lambda-list '(m))
(cl:defmethod recognized_id-val ((m <FaceEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-msg:recognized_id-val is deprecated.  Use pi_face_tracker-msg:recognized_id instead.")
  (recognized_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceEvent>) ostream)
  "Serializes a message object of type '<FaceEvent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'face_event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'face_event))
  (cl:let* ((signed (cl:slot-value msg 'face_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'recognized_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'recognized_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceEvent>) istream)
  "Deserializes a message object of type '<FaceEvent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'face_event) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'face_event) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'face_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'recognized_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'recognized_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceEvent>)))
  "Returns string type for a message object of type '<FaceEvent>"
  "pi_face_tracker/FaceEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceEvent)))
  "Returns string type for a message object of type 'FaceEvent"
  "pi_face_tracker/FaceEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceEvent>)))
  "Returns md5sum for a message object of type '<FaceEvent>"
  "6c1606bf80cc90b3ff678afd427716f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceEvent)))
  "Returns md5sum for a message object of type 'FaceEvent"
  "6c1606bf80cc90b3ff678afd427716f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceEvent>)))
  "Returns full string definition for message of type '<FaceEvent>"
  (cl:format cl:nil "string face_event~%int32 face_id~%string recognized_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceEvent)))
  "Returns full string definition for message of type 'FaceEvent"
  (cl:format cl:nil "string face_event~%int32 face_id~%string recognized_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceEvent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'face_event))
     4
     4 (cl:length (cl:slot-value msg 'recognized_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceEvent
    (cl:cons ':face_event (face_event msg))
    (cl:cons ':face_id (face_id msg))
    (cl:cons ':recognized_id (recognized_id msg))
))
