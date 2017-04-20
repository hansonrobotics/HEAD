; Auto-generated. Do not edit!


(cl:in-package chatbot-msg)


;//! \htmlinclude ChatMessage.msg.html

(cl:defclass <ChatMessage> (roslisp-msg-protocol:ros-message)
  ((utterance
    :reader utterance
    :initarg :utterance
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ChatMessage (<ChatMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChatMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChatMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chatbot-msg:<ChatMessage> is deprecated: use chatbot-msg:ChatMessage instead.")))

(cl:ensure-generic-function 'utterance-val :lambda-list '(m))
(cl:defmethod utterance-val ((m <ChatMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chatbot-msg:utterance-val is deprecated.  Use chatbot-msg:utterance instead.")
  (utterance m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <ChatMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chatbot-msg:confidence-val is deprecated.  Use chatbot-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChatMessage>) ostream)
  "Serializes a message object of type '<ChatMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'utterance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'utterance))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChatMessage>) istream)
  "Deserializes a message object of type '<ChatMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utterance) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'utterance) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChatMessage>)))
  "Returns string type for a message object of type '<ChatMessage>"
  "chatbot/ChatMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChatMessage)))
  "Returns string type for a message object of type 'ChatMessage"
  "chatbot/ChatMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChatMessage>)))
  "Returns md5sum for a message object of type '<ChatMessage>"
  "522e7b49288c831e97c7f4e96555af58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChatMessage)))
  "Returns md5sum for a message object of type 'ChatMessage"
  "522e7b49288c831e97c7f4e96555af58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChatMessage>)))
  "Returns full string definition for message of type '<ChatMessage>"
  (cl:format cl:nil "string utterance~%uint8 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChatMessage)))
  "Returns full string definition for message of type 'ChatMessage"
  (cl:format cl:nil "string utterance~%uint8 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChatMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'utterance))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChatMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'ChatMessage
    (cl:cons ':utterance (utterance msg))
    (cl:cons ':confidence (confidence msg))
))
