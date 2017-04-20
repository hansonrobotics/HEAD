; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude Visemes.msg.html

(cl:defclass <Visemes> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector blender_api_msgs-msg:Viseme)
   :initform (cl:make-array 0 :element-type 'blender_api_msgs-msg:Viseme :initial-element (cl:make-instance 'blender_api_msgs-msg:Viseme))))
)

(cl:defclass Visemes (<Visemes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Visemes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Visemes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<Visemes> is deprecated: use blender_api_msgs-msg:Visemes instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Visemes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:data-val is deprecated.  Use blender_api_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Visemes>) ostream)
  "Serializes a message object of type '<Visemes>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Visemes>) istream)
  "Deserializes a message object of type '<Visemes>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'blender_api_msgs-msg:Viseme))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Visemes>)))
  "Returns string type for a message object of type '<Visemes>"
  "blender_api_msgs/Visemes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Visemes)))
  "Returns string type for a message object of type 'Visemes"
  "blender_api_msgs/Visemes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Visemes>)))
  "Returns md5sum for a message object of type '<Visemes>"
  "9e70ce7b4191163ea9b52bbb867c3490")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Visemes)))
  "Returns md5sum for a message object of type 'Visemes"
  "9e70ce7b4191163ea9b52bbb867c3490")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Visemes>)))
  "Returns full string definition for message of type '<Visemes>"
  (cl:format cl:nil "Viseme[] data~%~%================================================================================~%MSG: blender_api_msgs/Viseme~%string name~%time start~%duration duration~%float32 rampin~%float32 rampout~%float32 magnitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Visemes)))
  "Returns full string definition for message of type 'Visemes"
  (cl:format cl:nil "Viseme[] data~%~%================================================================================~%MSG: blender_api_msgs/Viseme~%string name~%time start~%duration duration~%float32 rampin~%float32 rampout~%float32 magnitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Visemes>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Visemes>))
  "Converts a ROS message object to a list"
  (cl:list 'Visemes
    (cl:cons ':data (data msg))
))
