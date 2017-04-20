; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude Stop-request.msg.html

(cl:defclass <Stop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Stop-request (<Stop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Stop-request> is deprecated: use performances-srv:Stop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stop-request>) ostream)
  "Serializes a message object of type '<Stop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stop-request>) istream)
  "Deserializes a message object of type '<Stop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stop-request>)))
  "Returns string type for a service object of type '<Stop-request>"
  "performances/StopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop-request)))
  "Returns string type for a service object of type 'Stop-request"
  "performances/StopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stop-request>)))
  "Returns md5sum for a message object of type '<Stop-request>"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stop-request)))
  "Returns md5sum for a message object of type 'Stop-request"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stop-request>)))
  "Returns full string definition for message of type '<Stop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stop-request)))
  "Returns full string definition for message of type 'Stop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Stop-request
))
;//! \htmlinclude Stop-response.msg.html

(cl:defclass <Stop-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass Stop-response (<Stop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Stop-response> is deprecated: use performances-srv:Stop-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Stop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Stop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:time-val is deprecated.  Use performances-srv:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stop-response>) ostream)
  "Serializes a message object of type '<Stop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stop-response>) istream)
  "Deserializes a message object of type '<Stop-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stop-response>)))
  "Returns string type for a service object of type '<Stop-response>"
  "performances/StopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop-response)))
  "Returns string type for a service object of type 'Stop-response"
  "performances/StopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stop-response>)))
  "Returns md5sum for a message object of type '<Stop-response>"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stop-response)))
  "Returns md5sum for a message object of type 'Stop-response"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stop-response>)))
  "Returns full string definition for message of type '<Stop-response>"
  (cl:format cl:nil "bool success~%float64 time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stop-response)))
  "Returns full string definition for message of type 'Stop-response"
  (cl:format cl:nil "bool success~%float64 time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stop-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Stop-response
    (cl:cons ':success (success msg))
    (cl:cons ':time (time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Stop)))
  'Stop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Stop)))
  'Stop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stop)))
  "Returns string type for a service object of type '<Stop>"
  "performances/Stop")