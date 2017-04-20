; Auto-generated. Do not edit!


(cl:in-package webui-srv)


;//! \htmlinclude Json-request.msg.html

(cl:defclass <Json-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:string
    :initform ""))
)

(cl:defclass Json-request (<Json-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Json-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Json-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<Json-request> is deprecated: use webui-srv:Json-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <Json-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:request-val is deprecated.  Use webui-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Json-request>) ostream)
  "Serializes a message object of type '<Json-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Json-request>) istream)
  "Deserializes a message object of type '<Json-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Json-request>)))
  "Returns string type for a service object of type '<Json-request>"
  "webui/JsonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Json-request)))
  "Returns string type for a service object of type 'Json-request"
  "webui/JsonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Json-request>)))
  "Returns md5sum for a message object of type '<Json-request>"
  "1b26fc48cb9c9a0d4d515b6c472616de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Json-request)))
  "Returns md5sum for a message object of type 'Json-request"
  "1b26fc48cb9c9a0d4d515b6c472616de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Json-request>)))
  "Returns full string definition for message of type '<Json-request>"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Json-request)))
  "Returns full string definition for message of type 'Json-request"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Json-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Json-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Json-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude Json-response.msg.html

(cl:defclass <Json-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass Json-response (<Json-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Json-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Json-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name webui-srv:<Json-response> is deprecated: use webui-srv:Json-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Json-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:success-val is deprecated.  Use webui-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <Json-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader webui-srv:response-val is deprecated.  Use webui-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Json-response>) ostream)
  "Serializes a message object of type '<Json-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Json-response>) istream)
  "Deserializes a message object of type '<Json-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Json-response>)))
  "Returns string type for a service object of type '<Json-response>"
  "webui/JsonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Json-response)))
  "Returns string type for a service object of type 'Json-response"
  "webui/JsonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Json-response>)))
  "Returns md5sum for a message object of type '<Json-response>"
  "1b26fc48cb9c9a0d4d515b6c472616de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Json-response)))
  "Returns md5sum for a message object of type 'Json-response"
  "1b26fc48cb9c9a0d4d515b6c472616de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Json-response>)))
  "Returns full string definition for message of type '<Json-response>"
  (cl:format cl:nil "bool success~%string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Json-response)))
  "Returns full string definition for message of type 'Json-response"
  (cl:format cl:nil "bool success~%string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Json-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Json-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Json-response
    (cl:cons ':success (success msg))
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Json)))
  'Json-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Json)))
  'Json-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Json)))
  "Returns string type for a service object of type '<Json>"
  "webui/Json")