; Auto-generated. Do not edit!


(cl:in-package tts-srv)


;//! \htmlinclude TTSLength-request.msg.html

(cl:defclass <TTSLength-request> (roslisp-msg-protocol:ros-message)
  ((txt
    :reader txt
    :initarg :txt
    :type cl:string
    :initform "")
   (lang
    :reader lang
    :initarg :lang
    :type cl:string
    :initform ""))
)

(cl:defclass TTSLength-request (<TTSLength-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TTSLength-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TTSLength-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tts-srv:<TTSLength-request> is deprecated: use tts-srv:TTSLength-request instead.")))

(cl:ensure-generic-function 'txt-val :lambda-list '(m))
(cl:defmethod txt-val ((m <TTSLength-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tts-srv:txt-val is deprecated.  Use tts-srv:txt instead.")
  (txt m))

(cl:ensure-generic-function 'lang-val :lambda-list '(m))
(cl:defmethod lang-val ((m <TTSLength-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tts-srv:lang-val is deprecated.  Use tts-srv:lang instead.")
  (lang m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TTSLength-request>) ostream)
  "Serializes a message object of type '<TTSLength-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'txt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'txt))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'lang))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'lang))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TTSLength-request>) istream)
  "Deserializes a message object of type '<TTSLength-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'txt) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'txt) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lang) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'lang) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TTSLength-request>)))
  "Returns string type for a service object of type '<TTSLength-request>"
  "tts/TTSLengthRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTSLength-request)))
  "Returns string type for a service object of type 'TTSLength-request"
  "tts/TTSLengthRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TTSLength-request>)))
  "Returns md5sum for a message object of type '<TTSLength-request>"
  "1d9d61cca01d099b421f450f3266e6fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TTSLength-request)))
  "Returns md5sum for a message object of type 'TTSLength-request"
  "1d9d61cca01d099b421f450f3266e6fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TTSLength-request>)))
  "Returns full string definition for message of type '<TTSLength-request>"
  (cl:format cl:nil "string txt~%string lang~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TTSLength-request)))
  "Returns full string definition for message of type 'TTSLength-request"
  (cl:format cl:nil "string txt~%string lang~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TTSLength-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'txt))
     4 (cl:length (cl:slot-value msg 'lang))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TTSLength-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TTSLength-request
    (cl:cons ':txt (txt msg))
    (cl:cons ':lang (lang msg))
))
;//! \htmlinclude TTSLength-response.msg.html

(cl:defclass <TTSLength-response> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0))
)

(cl:defclass TTSLength-response (<TTSLength-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TTSLength-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TTSLength-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tts-srv:<TTSLength-response> is deprecated: use tts-srv:TTSLength-response instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <TTSLength-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tts-srv:length-val is deprecated.  Use tts-srv:length instead.")
  (length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TTSLength-response>) ostream)
  "Serializes a message object of type '<TTSLength-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TTSLength-response>) istream)
  "Deserializes a message object of type '<TTSLength-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TTSLength-response>)))
  "Returns string type for a service object of type '<TTSLength-response>"
  "tts/TTSLengthResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTSLength-response)))
  "Returns string type for a service object of type 'TTSLength-response"
  "tts/TTSLengthResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TTSLength-response>)))
  "Returns md5sum for a message object of type '<TTSLength-response>"
  "1d9d61cca01d099b421f450f3266e6fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TTSLength-response)))
  "Returns md5sum for a message object of type 'TTSLength-response"
  "1d9d61cca01d099b421f450f3266e6fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TTSLength-response>)))
  "Returns full string definition for message of type '<TTSLength-response>"
  (cl:format cl:nil "float32 length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TTSLength-response)))
  "Returns full string definition for message of type 'TTSLength-response"
  (cl:format cl:nil "float32 length~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TTSLength-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TTSLength-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TTSLength-response
    (cl:cons ':length (length msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TTSLength)))
  'TTSLength-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TTSLength)))
  'TTSLength-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TTSLength)))
  "Returns string type for a service object of type '<TTSLength>"
  "tts/TTSLength")