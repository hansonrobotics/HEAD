; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-srv)


;//! \htmlinclude GetAnimationLength-request.msg.html

(cl:defclass <GetAnimationLength-request> (roslisp-msg-protocol:ros-message)
  ((animation
    :reader animation
    :initarg :animation
    :type cl:string
    :initform ""))
)

(cl:defclass GetAnimationLength-request (<GetAnimationLength-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAnimationLength-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAnimationLength-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-srv:<GetAnimationLength-request> is deprecated: use blender_api_msgs-srv:GetAnimationLength-request instead.")))

(cl:ensure-generic-function 'animation-val :lambda-list '(m))
(cl:defmethod animation-val ((m <GetAnimationLength-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-srv:animation-val is deprecated.  Use blender_api_msgs-srv:animation instead.")
  (animation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAnimationLength-request>) ostream)
  "Serializes a message object of type '<GetAnimationLength-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'animation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'animation))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAnimationLength-request>) istream)
  "Deserializes a message object of type '<GetAnimationLength-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'animation) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'animation) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAnimationLength-request>)))
  "Returns string type for a service object of type '<GetAnimationLength-request>"
  "blender_api_msgs/GetAnimationLengthRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnimationLength-request)))
  "Returns string type for a service object of type 'GetAnimationLength-request"
  "blender_api_msgs/GetAnimationLengthRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAnimationLength-request>)))
  "Returns md5sum for a message object of type '<GetAnimationLength-request>"
  "fe4f383317a23749366d3cd09901e844")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAnimationLength-request)))
  "Returns md5sum for a message object of type 'GetAnimationLength-request"
  "fe4f383317a23749366d3cd09901e844")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAnimationLength-request>)))
  "Returns full string definition for message of type '<GetAnimationLength-request>"
  (cl:format cl:nil "string animation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAnimationLength-request)))
  "Returns full string definition for message of type 'GetAnimationLength-request"
  (cl:format cl:nil "string animation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAnimationLength-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'animation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAnimationLength-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAnimationLength-request
    (cl:cons ':animation (animation msg))
))
;//! \htmlinclude GetAnimationLength-response.msg.html

(cl:defclass <GetAnimationLength-response> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetAnimationLength-response (<GetAnimationLength-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAnimationLength-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAnimationLength-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-srv:<GetAnimationLength-response> is deprecated: use blender_api_msgs-srv:GetAnimationLength-response instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <GetAnimationLength-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-srv:length-val is deprecated.  Use blender_api_msgs-srv:length instead.")
  (length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAnimationLength-response>) ostream)
  "Serializes a message object of type '<GetAnimationLength-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAnimationLength-response>) istream)
  "Deserializes a message object of type '<GetAnimationLength-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAnimationLength-response>)))
  "Returns string type for a service object of type '<GetAnimationLength-response>"
  "blender_api_msgs/GetAnimationLengthResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnimationLength-response)))
  "Returns string type for a service object of type 'GetAnimationLength-response"
  "blender_api_msgs/GetAnimationLengthResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAnimationLength-response>)))
  "Returns md5sum for a message object of type '<GetAnimationLength-response>"
  "fe4f383317a23749366d3cd09901e844")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAnimationLength-response)))
  "Returns md5sum for a message object of type 'GetAnimationLength-response"
  "fe4f383317a23749366d3cd09901e844")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAnimationLength-response>)))
  "Returns full string definition for message of type '<GetAnimationLength-response>"
  (cl:format cl:nil "float32 length~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAnimationLength-response)))
  "Returns full string definition for message of type 'GetAnimationLength-response"
  (cl:format cl:nil "float32 length~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAnimationLength-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAnimationLength-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAnimationLength-response
    (cl:cons ':length (length msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAnimationLength)))
  'GetAnimationLength-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAnimationLength)))
  'GetAnimationLength-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAnimationLength)))
  "Returns string type for a service object of type '<GetAnimationLength>"
  "blender_api_msgs/GetAnimationLength")