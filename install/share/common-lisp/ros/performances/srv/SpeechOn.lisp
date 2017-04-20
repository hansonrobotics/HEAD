; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude SpeechOn-request.msg.html

(cl:defclass <SpeechOn-request> (roslisp-msg-protocol:ros-message)
  ((speech
    :reader speech
    :initarg :speech
    :type cl:string
    :initform ""))
)

(cl:defclass SpeechOn-request (<SpeechOn-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechOn-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechOn-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<SpeechOn-request> is deprecated: use performances-srv:SpeechOn-request instead.")))

(cl:ensure-generic-function 'speech-val :lambda-list '(m))
(cl:defmethod speech-val ((m <SpeechOn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:speech-val is deprecated.  Use performances-srv:speech instead.")
  (speech m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechOn-request>) ostream)
  "Serializes a message object of type '<SpeechOn-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'speech))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'speech))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechOn-request>) istream)
  "Deserializes a message object of type '<SpeechOn-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'speech) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechOn-request>)))
  "Returns string type for a service object of type '<SpeechOn-request>"
  "performances/SpeechOnRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechOn-request)))
  "Returns string type for a service object of type 'SpeechOn-request"
  "performances/SpeechOnRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechOn-request>)))
  "Returns md5sum for a message object of type '<SpeechOn-request>"
  "5e0f7ed055cc5952042284a15e187ee8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechOn-request)))
  "Returns md5sum for a message object of type 'SpeechOn-request"
  "5e0f7ed055cc5952042284a15e187ee8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechOn-request>)))
  "Returns full string definition for message of type '<SpeechOn-request>"
  (cl:format cl:nil "string speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechOn-request)))
  "Returns full string definition for message of type 'SpeechOn-request"
  (cl:format cl:nil "string speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechOn-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'speech))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechOn-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechOn-request
    (cl:cons ':speech (speech msg))
))
;//! \htmlinclude SpeechOn-response.msg.html

(cl:defclass <SpeechOn-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SpeechOn-response (<SpeechOn-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechOn-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechOn-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<SpeechOn-response> is deprecated: use performances-srv:SpeechOn-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SpeechOn-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechOn-response>) ostream)
  "Serializes a message object of type '<SpeechOn-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechOn-response>) istream)
  "Deserializes a message object of type '<SpeechOn-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechOn-response>)))
  "Returns string type for a service object of type '<SpeechOn-response>"
  "performances/SpeechOnResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechOn-response)))
  "Returns string type for a service object of type 'SpeechOn-response"
  "performances/SpeechOnResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechOn-response>)))
  "Returns md5sum for a message object of type '<SpeechOn-response>"
  "5e0f7ed055cc5952042284a15e187ee8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechOn-response)))
  "Returns md5sum for a message object of type 'SpeechOn-response"
  "5e0f7ed055cc5952042284a15e187ee8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechOn-response>)))
  "Returns full string definition for message of type '<SpeechOn-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechOn-response)))
  "Returns full string definition for message of type 'SpeechOn-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechOn-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechOn-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechOn-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpeechOn)))
  'SpeechOn-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpeechOn)))
  'SpeechOn-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechOn)))
  "Returns string type for a service object of type '<SpeechOn>"
  "performances/SpeechOn")