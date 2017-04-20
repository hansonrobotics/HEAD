; Auto-generated. Do not edit!


(cl:in-package webui-srv)


;//! \htmlinclude NodeDescription-request.msg.html

(cl:defclass <NodeDescription-request> (roslisp-msg-protocol:ros-message)
  ((node
    :reader node
    :initarg :node
    :type cl:string
    :initform ""))
)

(cl:defclass NodeDescription-request (<NodeDescription-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeDescription-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeDescription-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<NodeDescription-request> is deprecated: use webui-srv:NodeDescription-request instead.")))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <NodeDescription-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:node-val is deprecated.  Use webui-srv:node instead.")
  (node m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeDescription-request>) ostream)
  "Serializes a message object of type '<NodeDescription-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'node))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeDescription-request>) istream)
  "Deserializes a message object of type '<NodeDescription-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeDescription-request>)))
  "Returns string type for a service object of type '<NodeDescription-request>"
  "webui/NodeDescriptionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeDescription-request)))
  "Returns string type for a service object of type 'NodeDescription-request"
  "webui/NodeDescriptionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeDescription-request>)))
  "Returns md5sum for a message object of type '<NodeDescription-request>"
  "02f6d2fdd12a11088c4d10303fc40b3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeDescription-request)))
  "Returns md5sum for a message object of type 'NodeDescription-request"
  "02f6d2fdd12a11088c4d10303fc40b3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeDescription-request>)))
  "Returns full string definition for message of type '<NodeDescription-request>"
  (cl:format cl:nil "string node~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeDescription-request)))
  "Returns full string definition for message of type 'NodeDescription-request"
  (cl:format cl:nil "string node~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeDescription-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'node))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeDescription-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeDescription-request
    (cl:cons ':node (node msg))
))
;//! \htmlinclude NodeDescription-response.msg.html

(cl:defclass <NodeDescription-response> (roslisp-msg-protocol:ros-message)
  ((description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass NodeDescription-response (<NodeDescription-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeDescription-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeDescription-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<NodeDescription-response> is deprecated: use webui-srv:NodeDescription-response instead.")))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <NodeDescription-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:description-val is deprecated.  Use webui-srv:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeDescription-response>) ostream)
  "Serializes a message object of type '<NodeDescription-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeDescription-response>) istream)
  "Deserializes a message object of type '<NodeDescription-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeDescription-response>)))
  "Returns string type for a service object of type '<NodeDescription-response>"
  "webui/NodeDescriptionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeDescription-response)))
  "Returns string type for a service object of type 'NodeDescription-response"
  "webui/NodeDescriptionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeDescription-response>)))
  "Returns md5sum for a message object of type '<NodeDescription-response>"
  "02f6d2fdd12a11088c4d10303fc40b3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeDescription-response)))
  "Returns md5sum for a message object of type 'NodeDescription-response"
  "02f6d2fdd12a11088c4d10303fc40b3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeDescription-response>)))
  "Returns full string definition for message of type '<NodeDescription-response>"
  (cl:format cl:nil "string description~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeDescription-response)))
  "Returns full string definition for message of type 'NodeDescription-response"
  (cl:format cl:nil "string description~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeDescription-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeDescription-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeDescription-response
    (cl:cons ':description (description msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NodeDescription)))
  'NodeDescription-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NodeDescription)))
  'NodeDescription-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeDescription)))
  "Returns string type for a service object of type '<NodeDescription>"
  "webui/NodeDescription")