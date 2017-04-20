; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude Gestures.msg.html

(cl:defclass <Gestures> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector blender_api_msgs-msg:Gesture)
   :initform (cl:make-array 0 :element-type 'blender_api_msgs-msg:Gesture :initial-element (cl:make-instance 'blender_api_msgs-msg:Gesture))))
)

(cl:defclass Gestures (<Gestures>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gestures>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gestures)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<Gestures> is deprecated: use blender_api_msgs-msg:Gestures instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Gestures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:data-val is deprecated.  Use blender_api_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gestures>) ostream)
  "Serializes a message object of type '<Gestures>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gestures>) istream)
  "Deserializes a message object of type '<Gestures>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'blender_api_msgs-msg:Gesture))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gestures>)))
  "Returns string type for a message object of type '<Gestures>"
  "blender_api_msgs/Gestures")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gestures)))
  "Returns string type for a message object of type 'Gestures"
  "blender_api_msgs/Gestures")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gestures>)))
  "Returns md5sum for a message object of type '<Gestures>"
  "77b3761bbffee13f6cb6a7a547fdb43c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gestures)))
  "Returns md5sum for a message object of type 'Gestures"
  "77b3761bbffee13f6cb6a7a547fdb43c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gestures>)))
  "Returns full string definition for message of type '<Gestures>"
  (cl:format cl:nil "Gesture[] data~%~%================================================================================~%MSG: blender_api_msgs/Gesture~%string name~%float32 speed~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gestures)))
  "Returns full string definition for message of type 'Gestures"
  (cl:format cl:nil "Gesture[] data~%~%================================================================================~%MSG: blender_api_msgs/Gesture~%string name~%float32 speed~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gestures>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gestures>))
  "Converts a ROS message object to a list"
  (cl:list 'Gestures
    (cl:cons ':data (data msg))
))
