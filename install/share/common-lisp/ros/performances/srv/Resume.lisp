; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude Resume-request.msg.html

(cl:defclass <Resume-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Resume-request (<Resume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Resume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Resume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Resume-request> is deprecated: use performances-srv:Resume-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Resume-request>) ostream)
  "Serializes a message object of type '<Resume-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Resume-request>) istream)
  "Deserializes a message object of type '<Resume-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Resume-request>)))
  "Returns string type for a service object of type '<Resume-request>"
  "performances/ResumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Resume-request)))
  "Returns string type for a service object of type 'Resume-request"
  "performances/ResumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Resume-request>)))
  "Returns md5sum for a message object of type '<Resume-request>"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Resume-request)))
  "Returns md5sum for a message object of type 'Resume-request"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Resume-request>)))
  "Returns full string definition for message of type '<Resume-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Resume-request)))
  "Returns full string definition for message of type 'Resume-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Resume-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Resume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Resume-request
))
;//! \htmlinclude Resume-response.msg.html

(cl:defclass <Resume-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Resume-response (<Resume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Resume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Resume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Resume-response> is deprecated: use performances-srv:Resume-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Resume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Resume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:time-val is deprecated.  Use performances-srv:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Resume-response>) ostream)
  "Serializes a message object of type '<Resume-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Resume-response>) istream)
  "Deserializes a message object of type '<Resume-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Resume-response>)))
  "Returns string type for a service object of type '<Resume-response>"
  "performances/ResumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Resume-response)))
  "Returns string type for a service object of type 'Resume-response"
  "performances/ResumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Resume-response>)))
  "Returns md5sum for a message object of type '<Resume-response>"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Resume-response)))
  "Returns md5sum for a message object of type 'Resume-response"
  "a1c819fef9d84d752cee0adc8561a9b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Resume-response>)))
  "Returns full string definition for message of type '<Resume-response>"
  (cl:format cl:nil "bool success~%float64 time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Resume-response)))
  "Returns full string definition for message of type 'Resume-response"
  (cl:format cl:nil "bool success~%float64 time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Resume-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Resume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Resume-response
    (cl:cons ':success (success msg))
    (cl:cons ':time (time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Resume)))
  'Resume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Resume)))
  'Resume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Resume)))
  "Returns string type for a service object of type '<Resume>"
  "performances/Resume")