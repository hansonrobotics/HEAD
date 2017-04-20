; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude Run-request.msg.html

(cl:defclass <Run-request> (roslisp-msg-protocol:ros-message)
  ((startTime
    :reader startTime
    :initarg :startTime
    :type cl:float
    :initform 0.0))
)

(cl:defclass Run-request (<Run-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Run-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Run-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Run-request> is deprecated: use performances-srv:Run-request instead.")))

(cl:ensure-generic-function 'startTime-val :lambda-list '(m))
(cl:defmethod startTime-val ((m <Run-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:startTime-val is deprecated.  Use performances-srv:startTime instead.")
  (startTime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Run-request>) ostream)
  "Serializes a message object of type '<Run-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'startTime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Run-request>) istream)
  "Deserializes a message object of type '<Run-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startTime) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Run-request>)))
  "Returns string type for a service object of type '<Run-request>"
  "performances/RunRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Run-request)))
  "Returns string type for a service object of type 'Run-request"
  "performances/RunRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Run-request>)))
  "Returns md5sum for a message object of type '<Run-request>"
  "722a4133f235106aa61195f0a0d0ebce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Run-request)))
  "Returns md5sum for a message object of type 'Run-request"
  "722a4133f235106aa61195f0a0d0ebce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Run-request>)))
  "Returns full string definition for message of type '<Run-request>"
  (cl:format cl:nil "float64 startTime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Run-request)))
  "Returns full string definition for message of type 'Run-request"
  (cl:format cl:nil "float64 startTime~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Run-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Run-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Run-request
    (cl:cons ':startTime (startTime msg))
))
;//! \htmlinclude Run-response.msg.html

(cl:defclass <Run-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Run-response (<Run-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Run-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Run-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Run-response> is deprecated: use performances-srv:Run-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Run-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Run-response>) ostream)
  "Serializes a message object of type '<Run-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Run-response>) istream)
  "Deserializes a message object of type '<Run-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Run-response>)))
  "Returns string type for a service object of type '<Run-response>"
  "performances/RunResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Run-response)))
  "Returns string type for a service object of type 'Run-response"
  "performances/RunResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Run-response>)))
  "Returns md5sum for a message object of type '<Run-response>"
  "722a4133f235106aa61195f0a0d0ebce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Run-response)))
  "Returns md5sum for a message object of type 'Run-response"
  "722a4133f235106aa61195f0a0d0ebce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Run-response>)))
  "Returns full string definition for message of type '<Run-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Run-response)))
  "Returns full string definition for message of type 'Run-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Run-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Run-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Run-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Run)))
  'Run-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Run)))
  'Run-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Run)))
  "Returns string type for a service object of type '<Run>"
  "performances/Run")