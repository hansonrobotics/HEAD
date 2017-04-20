; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude GetAPIVersion.msg.html

(cl:defclass <GetAPIVersion> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetAPIVersion (<GetAPIVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAPIVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAPIVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<GetAPIVersion> is deprecated: use blender_api_msgs-msg:GetAPIVersion instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetAPIVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:value-val is deprecated.  Use blender_api_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAPIVersion>) ostream)
  "Serializes a message object of type '<GetAPIVersion>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAPIVersion>) istream)
  "Deserializes a message object of type '<GetAPIVersion>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAPIVersion>)))
  "Returns string type for a message object of type '<GetAPIVersion>"
  "blender_api_msgs/GetAPIVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAPIVersion)))
  "Returns string type for a message object of type 'GetAPIVersion"
  "blender_api_msgs/GetAPIVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAPIVersion>)))
  "Returns md5sum for a message object of type '<GetAPIVersion>"
  "e4da51e86d3bac963ee2bb1f41031407")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAPIVersion)))
  "Returns md5sum for a message object of type 'GetAPIVersion"
  "e4da51e86d3bac963ee2bb1f41031407")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAPIVersion>)))
  "Returns full string definition for message of type '<GetAPIVersion>"
  (cl:format cl:nil "uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAPIVersion)))
  "Returns full string definition for message of type 'GetAPIVersion"
  (cl:format cl:nil "uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAPIVersion>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAPIVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAPIVersion
    (cl:cons ':value (value msg))
))
