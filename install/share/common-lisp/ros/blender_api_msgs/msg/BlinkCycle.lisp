; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude BlinkCycle.msg.html

(cl:defclass <BlinkCycle> (roslisp-msg-protocol:ros-message)
  ((mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0)
   (variation
    :reader variation
    :initarg :variation
    :type cl:float
    :initform 0.0))
)

(cl:defclass BlinkCycle (<BlinkCycle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlinkCycle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlinkCycle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<BlinkCycle> is deprecated: use blender_api_msgs-msg:BlinkCycle instead.")))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <BlinkCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:mean-val is deprecated.  Use blender_api_msgs-msg:mean instead.")
  (mean m))

(cl:ensure-generic-function 'variation-val :lambda-list '(m))
(cl:defmethod variation-val ((m <BlinkCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:variation-val is deprecated.  Use blender_api_msgs-msg:variation instead.")
  (variation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlinkCycle>) ostream)
  "Serializes a message object of type '<BlinkCycle>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'variation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlinkCycle>) istream)
  "Deserializes a message object of type '<BlinkCycle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'variation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlinkCycle>)))
  "Returns string type for a message object of type '<BlinkCycle>"
  "blender_api_msgs/BlinkCycle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlinkCycle)))
  "Returns string type for a message object of type 'BlinkCycle"
  "blender_api_msgs/BlinkCycle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlinkCycle>)))
  "Returns md5sum for a message object of type '<BlinkCycle>"
  "c18aa77e257bbab07fdf7bbe6ace1178")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlinkCycle)))
  "Returns md5sum for a message object of type 'BlinkCycle"
  "c18aa77e257bbab07fdf7bbe6ace1178")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlinkCycle>)))
  "Returns full string definition for message of type '<BlinkCycle>"
  (cl:format cl:nil "float32 mean~%float32 variation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlinkCycle)))
  "Returns full string definition for message of type 'BlinkCycle"
  (cl:format cl:nil "float32 mean~%float32 variation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlinkCycle>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlinkCycle>))
  "Converts a ROS message object to a list"
  (cl:list 'BlinkCycle
    (cl:cons ':mean (mean msg))
    (cl:cons ':variation (variation msg))
))
