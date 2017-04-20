; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-srv)


;//! \htmlinclude GetParam-request.msg.html

(cl:defclass <GetParam-request> (roslisp-msg-protocol:ros-message)
  ((param
    :reader param
    :initarg :param
    :type cl:string
    :initform ""))
)

(cl:defclass GetParam-request (<GetParam-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParam-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParam-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-srv:<GetParam-request> is deprecated: use blender_api_msgs-srv:GetParam-request instead.")))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <GetParam-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-srv:param-val is deprecated.  Use blender_api_msgs-srv:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParam-request>) ostream)
  "Serializes a message object of type '<GetParam-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParam-request>) istream)
  "Deserializes a message object of type '<GetParam-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParam-request>)))
  "Returns string type for a service object of type '<GetParam-request>"
  "blender_api_msgs/GetParamRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParam-request)))
  "Returns string type for a service object of type 'GetParam-request"
  "blender_api_msgs/GetParamRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParam-request>)))
  "Returns md5sum for a message object of type '<GetParam-request>"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParam-request)))
  "Returns md5sum for a message object of type 'GetParam-request"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParam-request>)))
  "Returns full string definition for message of type '<GetParam-request>"
  (cl:format cl:nil "string param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParam-request)))
  "Returns full string definition for message of type 'GetParam-request"
  (cl:format cl:nil "string param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParam-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'param))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParam-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParam-request
    (cl:cons ':param (param msg))
))
;//! \htmlinclude GetParam-response.msg.html

(cl:defclass <GetParam-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass GetParam-response (<GetParam-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetParam-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetParam-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-srv:<GetParam-response> is deprecated: use blender_api_msgs-srv:GetParam-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetParam-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-srv:value-val is deprecated.  Use blender_api_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetParam-response>) ostream)
  "Serializes a message object of type '<GetParam-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetParam-response>) istream)
  "Deserializes a message object of type '<GetParam-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetParam-response>)))
  "Returns string type for a service object of type '<GetParam-response>"
  "blender_api_msgs/GetParamResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParam-response)))
  "Returns string type for a service object of type 'GetParam-response"
  "blender_api_msgs/GetParamResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetParam-response>)))
  "Returns md5sum for a message object of type '<GetParam-response>"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetParam-response)))
  "Returns md5sum for a message object of type 'GetParam-response"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetParam-response>)))
  "Returns full string definition for message of type '<GetParam-response>"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetParam-response)))
  "Returns full string definition for message of type 'GetParam-response"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetParam-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetParam-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetParam-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetParam)))
  'GetParam-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetParam)))
  'GetParam-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetParam)))
  "Returns string type for a service object of type '<GetParam>"
  "blender_api_msgs/GetParam")