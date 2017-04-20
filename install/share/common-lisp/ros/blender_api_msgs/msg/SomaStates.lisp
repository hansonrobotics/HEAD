; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude SomaStates.msg.html

(cl:defclass <SomaStates> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector blender_api_msgs-msg:SomaState)
   :initform (cl:make-array 0 :element-type 'blender_api_msgs-msg:SomaState :initial-element (cl:make-instance 'blender_api_msgs-msg:SomaState))))
)

(cl:defclass SomaStates (<SomaStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SomaStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SomaStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<SomaStates> is deprecated: use blender_api_msgs-msg:SomaStates instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SomaStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:data-val is deprecated.  Use blender_api_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SomaStates>) ostream)
  "Serializes a message object of type '<SomaStates>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SomaStates>) istream)
  "Deserializes a message object of type '<SomaStates>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'blender_api_msgs-msg:SomaState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SomaStates>)))
  "Returns string type for a message object of type '<SomaStates>"
  "blender_api_msgs/SomaStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SomaStates)))
  "Returns string type for a message object of type 'SomaStates"
  "blender_api_msgs/SomaStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SomaStates>)))
  "Returns md5sum for a message object of type '<SomaStates>"
  "1951e17e32b03a8c08e010b7ed135424")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SomaStates)))
  "Returns md5sum for a message object of type 'SomaStates"
  "1951e17e32b03a8c08e010b7ed135424")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SomaStates>)))
  "Returns full string definition for message of type '<SomaStates>"
  (cl:format cl:nil "SomaState[] data~%~%================================================================================~%MSG: blender_api_msgs/SomaState~%string name~%float32 magnitude~%float32 rate~%duration ease_in~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SomaStates)))
  "Returns full string definition for message of type 'SomaStates"
  (cl:format cl:nil "SomaState[] data~%~%================================================================================~%MSG: blender_api_msgs/SomaState~%string name~%float32 magnitude~%float32 rate~%duration ease_in~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SomaStates>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SomaStates>))
  "Converts a ROS message object to a list"
  (cl:list 'SomaStates
    (cl:cons ':data (data msg))
))
