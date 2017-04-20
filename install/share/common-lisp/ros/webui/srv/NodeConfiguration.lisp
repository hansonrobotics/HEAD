; Auto-generated. Do not edit!


(cl:in-package webui-srv)


;//! \htmlinclude NodeConfiguration-request.msg.html

(cl:defclass <NodeConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((node
    :reader node
    :initarg :node
    :type cl:string
    :initform ""))
)

(cl:defclass NodeConfiguration-request (<NodeConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<NodeConfiguration-request> is deprecated: use webui-srv:NodeConfiguration-request instead.")))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <NodeConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:node-val is deprecated.  Use webui-srv:node instead.")
  (node m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeConfiguration-request>) ostream)
  "Serializes a message object of type '<NodeConfiguration-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeConfiguration-request>) istream)
  "Deserializes a message object of type '<NodeConfiguration-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'node) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeConfiguration-request>)))
  "Returns string type for a service object of type '<NodeConfiguration-request>"
  "webui/NodeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeConfiguration-request)))
  "Returns string type for a service object of type 'NodeConfiguration-request"
  "webui/NodeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeConfiguration-request>)))
  "Returns md5sum for a message object of type '<NodeConfiguration-request>"
  "7bbc3f18b639203023884204271c0124")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeConfiguration-request)))
  "Returns md5sum for a message object of type 'NodeConfiguration-request"
  "7bbc3f18b639203023884204271c0124")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeConfiguration-request>)))
  "Returns full string definition for message of type '<NodeConfiguration-request>"
  (cl:format cl:nil "string node~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeConfiguration-request)))
  "Returns full string definition for message of type 'NodeConfiguration-request"
  (cl:format cl:nil "string node~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeConfiguration-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeConfiguration-request
    (cl:cons ':node (node msg))
))
;//! \htmlinclude NodeConfiguration-response.msg.html

(cl:defclass <NodeConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((configuration
    :reader configuration
    :initarg :configuration
    :type cl:string
    :initform ""))
)

(cl:defclass NodeConfiguration-response (<NodeConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<NodeConfiguration-response> is deprecated: use webui-srv:NodeConfiguration-response instead.")))

(cl:ensure-generic-function 'configuration-val :lambda-list '(m))
(cl:defmethod configuration-val ((m <NodeConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:configuration-val is deprecated.  Use webui-srv:configuration instead.")
  (configuration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeConfiguration-response>) ostream)
  "Serializes a message object of type '<NodeConfiguration-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'configuration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'configuration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeConfiguration-response>) istream)
  "Deserializes a message object of type '<NodeConfiguration-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'configuration) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'configuration) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeConfiguration-response>)))
  "Returns string type for a service object of type '<NodeConfiguration-response>"
  "webui/NodeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeConfiguration-response)))
  "Returns string type for a service object of type 'NodeConfiguration-response"
  "webui/NodeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeConfiguration-response>)))
  "Returns md5sum for a message object of type '<NodeConfiguration-response>"
  "7bbc3f18b639203023884204271c0124")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeConfiguration-response)))
  "Returns md5sum for a message object of type 'NodeConfiguration-response"
  "7bbc3f18b639203023884204271c0124")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeConfiguration-response>)))
  "Returns full string definition for message of type '<NodeConfiguration-response>"
  (cl:format cl:nil "string configuration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeConfiguration-response)))
  "Returns full string definition for message of type 'NodeConfiguration-response"
  (cl:format cl:nil "string configuration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeConfiguration-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'configuration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeConfiguration-response
    (cl:cons ':configuration (configuration msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NodeConfiguration)))
  'NodeConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NodeConfiguration)))
  'NodeConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeConfiguration)))
  "Returns string type for a service object of type '<NodeConfiguration>"
  "webui/NodeConfiguration")