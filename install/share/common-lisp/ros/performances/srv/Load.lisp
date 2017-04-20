; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude Load-request.msg.html

(cl:defclass <Load-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass Load-request (<Load-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Load-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Load-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Load-request> is deprecated: use performances-srv:Load-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Load-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:id-val is deprecated.  Use performances-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Load-request>) ostream)
  "Serializes a message object of type '<Load-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Load-request>) istream)
  "Deserializes a message object of type '<Load-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Load-request>)))
  "Returns string type for a service object of type '<Load-request>"
  "performances/LoadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Load-request)))
  "Returns string type for a service object of type 'Load-request"
  "performances/LoadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Load-request>)))
  "Returns md5sum for a message object of type '<Load-request>"
  "3cf9ab71d8d41cf259dd332e6f7c0a24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Load-request)))
  "Returns md5sum for a message object of type 'Load-request"
  "3cf9ab71d8d41cf259dd332e6f7c0a24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Load-request>)))
  "Returns full string definition for message of type '<Load-request>"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Load-request)))
  "Returns full string definition for message of type 'Load-request"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Load-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Load-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Load-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude Load-response.msg.html

(cl:defclass <Load-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (nodes
    :reader nodes
    :initarg :nodes
    :type cl:string
    :initform ""))
)

(cl:defclass Load-response (<Load-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Load-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Load-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Load-response> is deprecated: use performances-srv:Load-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Load-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <Load-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:nodes-val is deprecated.  Use performances-srv:nodes instead.")
  (nodes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Load-response>) ostream)
  "Serializes a message object of type '<Load-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Load-response>) istream)
  "Deserializes a message object of type '<Load-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Load-response>)))
  "Returns string type for a service object of type '<Load-response>"
  "performances/LoadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Load-response)))
  "Returns string type for a service object of type 'Load-response"
  "performances/LoadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Load-response>)))
  "Returns md5sum for a message object of type '<Load-response>"
  "3cf9ab71d8d41cf259dd332e6f7c0a24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Load-response)))
  "Returns md5sum for a message object of type 'Load-response"
  "3cf9ab71d8d41cf259dd332e6f7c0a24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Load-response>)))
  "Returns full string definition for message of type '<Load-response>"
  (cl:format cl:nil "bool success~%string nodes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Load-response)))
  "Returns full string definition for message of type 'Load-response"
  (cl:format cl:nil "bool success~%string nodes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Load-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'nodes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Load-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Load-response
    (cl:cons ':success (success msg))
    (cl:cons ':nodes (nodes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Load)))
  'Load-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Load)))
  'Load-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Load)))
  "Returns string type for a service object of type '<Load>"
  "performances/Load")