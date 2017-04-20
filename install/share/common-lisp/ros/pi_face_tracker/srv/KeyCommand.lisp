; Auto-generated. Do not edit!


(cl:in-package pi_face_tracker-srv)


;//! \htmlinclude KeyCommand-request.msg.html

(cl:defclass <KeyCommand-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass KeyCommand-request (<KeyCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-srv:<KeyCommand-request> is deprecated: use pi_face_tracker-srv:KeyCommand-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <KeyCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-srv:command-val is deprecated.  Use pi_face_tracker-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyCommand-request>) ostream)
  "Serializes a message object of type '<KeyCommand-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyCommand-request>) istream)
  "Deserializes a message object of type '<KeyCommand-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyCommand-request>)))
  "Returns string type for a service object of type '<KeyCommand-request>"
  "pi_face_tracker/KeyCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyCommand-request)))
  "Returns string type for a service object of type 'KeyCommand-request"
  "pi_face_tracker/KeyCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyCommand-request>)))
  "Returns md5sum for a message object of type '<KeyCommand-request>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyCommand-request)))
  "Returns md5sum for a message object of type 'KeyCommand-request"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyCommand-request>)))
  "Returns full string definition for message of type '<KeyCommand-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyCommand-request)))
  "Returns full string definition for message of type 'KeyCommand-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyCommand-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyCommand-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude KeyCommand-response.msg.html

(cl:defclass <KeyCommand-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass KeyCommand-response (<KeyCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-srv:<KeyCommand-response> is deprecated: use pi_face_tracker-srv:KeyCommand-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyCommand-response>) ostream)
  "Serializes a message object of type '<KeyCommand-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyCommand-response>) istream)
  "Deserializes a message object of type '<KeyCommand-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyCommand-response>)))
  "Returns string type for a service object of type '<KeyCommand-response>"
  "pi_face_tracker/KeyCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyCommand-response)))
  "Returns string type for a service object of type 'KeyCommand-response"
  "pi_face_tracker/KeyCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyCommand-response>)))
  "Returns md5sum for a message object of type '<KeyCommand-response>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyCommand-response)))
  "Returns md5sum for a message object of type 'KeyCommand-response"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyCommand-response>)))
  "Returns full string definition for message of type '<KeyCommand-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyCommand-response)))
  "Returns full string definition for message of type 'KeyCommand-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyCommand-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyCommand-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'KeyCommand)))
  'KeyCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'KeyCommand)))
  'KeyCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyCommand)))
  "Returns string type for a service object of type '<KeyCommand>"
  "pi_face_tracker/KeyCommand")