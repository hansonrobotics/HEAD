; Auto-generated. Do not edit!


(cl:in-package webui-srv)


;//! \htmlinclude ConfigurableNodes-request.msg.html

(cl:defclass <ConfigurableNodes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConfigurableNodes-request (<ConfigurableNodes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurableNodes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurableNodes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<ConfigurableNodes-request> is deprecated: use webui-srv:ConfigurableNodes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurableNodes-request>) ostream)
  "Serializes a message object of type '<ConfigurableNodes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurableNodes-request>) istream)
  "Deserializes a message object of type '<ConfigurableNodes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurableNodes-request>)))
  "Returns string type for a service object of type '<ConfigurableNodes-request>"
  "webui/ConfigurableNodesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurableNodes-request)))
  "Returns string type for a service object of type 'ConfigurableNodes-request"
  "webui/ConfigurableNodesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurableNodes-request>)))
  "Returns md5sum for a message object of type '<ConfigurableNodes-request>"
  "3d07bfda1268b4f76b16b7ba8a82665d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurableNodes-request)))
  "Returns md5sum for a message object of type 'ConfigurableNodes-request"
  "3d07bfda1268b4f76b16b7ba8a82665d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurableNodes-request>)))
  "Returns full string definition for message of type '<ConfigurableNodes-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurableNodes-request)))
  "Returns full string definition for message of type 'ConfigurableNodes-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurableNodes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurableNodes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurableNodes-request
))
;//! \htmlinclude ConfigurableNodes-response.msg.html

(cl:defclass <ConfigurableNodes-response> (roslisp-msg-protocol:ros-message)
  ((nodes
    :reader nodes
    :initarg :nodes
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ConfigurableNodes-response (<ConfigurableNodes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurableNodes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurableNodes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<ConfigurableNodes-response> is deprecated: use webui-srv:ConfigurableNodes-response instead.")))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <ConfigurableNodes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:nodes-val is deprecated.  Use webui-srv:nodes instead.")
  (nodes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurableNodes-response>) ostream)
  "Serializes a message object of type '<ConfigurableNodes-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'nodes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurableNodes-response>) istream)
  "Deserializes a message object of type '<ConfigurableNodes-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurableNodes-response>)))
  "Returns string type for a service object of type '<ConfigurableNodes-response>"
  "webui/ConfigurableNodesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurableNodes-response)))
  "Returns string type for a service object of type 'ConfigurableNodes-response"
  "webui/ConfigurableNodesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurableNodes-response>)))
  "Returns md5sum for a message object of type '<ConfigurableNodes-response>"
  "3d07bfda1268b4f76b16b7ba8a82665d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurableNodes-response)))
  "Returns md5sum for a message object of type 'ConfigurableNodes-response"
  "3d07bfda1268b4f76b16b7ba8a82665d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurableNodes-response>)))
  "Returns full string definition for message of type '<ConfigurableNodes-response>"
  (cl:format cl:nil "string[] nodes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurableNodes-response)))
  "Returns full string definition for message of type 'ConfigurableNodes-response"
  (cl:format cl:nil "string[] nodes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurableNodes-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurableNodes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurableNodes-response
    (cl:cons ':nodes (nodes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigurableNodes)))
  'ConfigurableNodes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigurableNodes)))
  'ConfigurableNodes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurableNodes)))
  "Returns string type for a service object of type '<ConfigurableNodes>"
  "webui/ConfigurableNodes")