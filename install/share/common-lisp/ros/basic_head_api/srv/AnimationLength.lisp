; Auto-generated. Do not edit!


(cl:in-package basic_head_api-srv)


;//! \htmlinclude AnimationLength-request.msg.html

(cl:defclass <AnimationLength-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass AnimationLength-request (<AnimationLength-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationLength-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationLength-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-srv:<AnimationLength-request> is deprecated: use basic_head_api-srv:AnimationLength-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <AnimationLength-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-srv:name-val is deprecated.  Use basic_head_api-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationLength-request>) ostream)
  "Serializes a message object of type '<AnimationLength-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationLength-request>) istream)
  "Deserializes a message object of type '<AnimationLength-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationLength-request>)))
  "Returns string type for a service object of type '<AnimationLength-request>"
  "basic_head_api/AnimationLengthRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationLength-request)))
  "Returns string type for a service object of type 'AnimationLength-request"
  "basic_head_api/AnimationLengthRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationLength-request>)))
  "Returns md5sum for a message object of type '<AnimationLength-request>"
  "a312124afea9a9a7b783b7b50a582e0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationLength-request)))
  "Returns md5sum for a message object of type 'AnimationLength-request"
  "a312124afea9a9a7b783b7b50a582e0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationLength-request>)))
  "Returns full string definition for message of type '<AnimationLength-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationLength-request)))
  "Returns full string definition for message of type 'AnimationLength-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationLength-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationLength-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationLength-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude AnimationLength-response.msg.html

(cl:defclass <AnimationLength-response> (roslisp-msg-protocol:ros-message)
  ((frames
    :reader frames
    :initarg :frames
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnimationLength-response (<AnimationLength-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnimationLength-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnimationLength-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-srv:<AnimationLength-response> is deprecated: use basic_head_api-srv:AnimationLength-response instead.")))

(cl:ensure-generic-function 'frames-val :lambda-list '(m))
(cl:defmethod frames-val ((m <AnimationLength-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-srv:frames-val is deprecated.  Use basic_head_api-srv:frames instead.")
  (frames m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnimationLength-response>) ostream)
  "Serializes a message object of type '<AnimationLength-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frames)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnimationLength-response>) istream)
  "Deserializes a message object of type '<AnimationLength-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frames)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnimationLength-response>)))
  "Returns string type for a service object of type '<AnimationLength-response>"
  "basic_head_api/AnimationLengthResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationLength-response)))
  "Returns string type for a service object of type 'AnimationLength-response"
  "basic_head_api/AnimationLengthResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnimationLength-response>)))
  "Returns md5sum for a message object of type '<AnimationLength-response>"
  "a312124afea9a9a7b783b7b50a582e0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnimationLength-response)))
  "Returns md5sum for a message object of type 'AnimationLength-response"
  "a312124afea9a9a7b783b7b50a582e0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnimationLength-response>)))
  "Returns full string definition for message of type '<AnimationLength-response>"
  (cl:format cl:nil "uint16 frames~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnimationLength-response)))
  "Returns full string definition for message of type 'AnimationLength-response"
  (cl:format cl:nil "uint16 frames~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnimationLength-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnimationLength-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AnimationLength-response
    (cl:cons ':frames (frames msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AnimationLength)))
  'AnimationLength-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AnimationLength)))
  'AnimationLength-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnimationLength)))
  "Returns string type for a service object of type '<AnimationLength>"
  "basic_head_api/AnimationLength")