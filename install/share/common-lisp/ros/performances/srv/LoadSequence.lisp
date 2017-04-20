; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude LoadSequence-request.msg.html

(cl:defclass <LoadSequence-request> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass LoadSequence-request (<LoadSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<LoadSequence-request> is deprecated: use performances-srv:LoadSequence-request instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <LoadSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:ids-val is deprecated.  Use performances-srv:ids instead.")
  (ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadSequence-request>) ostream)
  "Serializes a message object of type '<LoadSequence-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadSequence-request>) istream)
  "Deserializes a message object of type '<LoadSequence-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadSequence-request>)))
  "Returns string type for a service object of type '<LoadSequence-request>"
  "performances/LoadSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadSequence-request)))
  "Returns string type for a service object of type 'LoadSequence-request"
  "performances/LoadSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadSequence-request>)))
  "Returns md5sum for a message object of type '<LoadSequence-request>"
  "732273dbd23c7938c375ea61c340dd4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadSequence-request)))
  "Returns md5sum for a message object of type 'LoadSequence-request"
  "732273dbd23c7938c375ea61c340dd4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadSequence-request>)))
  "Returns full string definition for message of type '<LoadSequence-request>"
  (cl:format cl:nil "string[] ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadSequence-request)))
  "Returns full string definition for message of type 'LoadSequence-request"
  (cl:format cl:nil "string[] ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadSequence-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadSequence-request
    (cl:cons ':ids (ids msg))
))
;//! \htmlinclude LoadSequence-response.msg.html

(cl:defclass <LoadSequence-response> (roslisp-msg-protocol:ros-message)
  ((nodes
    :reader nodes
    :initarg :nodes
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadSequence-response (<LoadSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<LoadSequence-response> is deprecated: use performances-srv:LoadSequence-response instead.")))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <LoadSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:nodes-val is deprecated.  Use performances-srv:nodes instead.")
  (nodes m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadSequence-response>) ostream)
  "Serializes a message object of type '<LoadSequence-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodes))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadSequence-response>) istream)
  "Deserializes a message object of type '<LoadSequence-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadSequence-response>)))
  "Returns string type for a service object of type '<LoadSequence-response>"
  "performances/LoadSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadSequence-response)))
  "Returns string type for a service object of type 'LoadSequence-response"
  "performances/LoadSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadSequence-response>)))
  "Returns md5sum for a message object of type '<LoadSequence-response>"
  "732273dbd23c7938c375ea61c340dd4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadSequence-response)))
  "Returns md5sum for a message object of type 'LoadSequence-response"
  "732273dbd23c7938c375ea61c340dd4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadSequence-response>)))
  "Returns full string definition for message of type '<LoadSequence-response>"
  (cl:format cl:nil "string nodes~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadSequence-response)))
  "Returns full string definition for message of type 'LoadSequence-response"
  (cl:format cl:nil "string nodes~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadSequence-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'nodes))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadSequence-response
    (cl:cons ':nodes (nodes msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadSequence)))
  'LoadSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadSequence)))
  'LoadSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadSequence)))
  "Returns string type for a service object of type '<LoadSequence>"
  "performances/LoadSequence")