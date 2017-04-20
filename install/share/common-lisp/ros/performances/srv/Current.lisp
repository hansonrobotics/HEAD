; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude Current-request.msg.html

(cl:defclass <Current-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Current-request (<Current-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Current-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Current-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Current-request> is deprecated: use performances-srv:Current-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Current-request>) ostream)
  "Serializes a message object of type '<Current-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Current-request>) istream)
  "Deserializes a message object of type '<Current-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Current-request>)))
  "Returns string type for a service object of type '<Current-request>"
  "performances/CurrentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Current-request)))
  "Returns string type for a service object of type 'Current-request"
  "performances/CurrentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Current-request>)))
  "Returns md5sum for a message object of type '<Current-request>"
  "60f910a2f5cc14b982946ccdb2fea967")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Current-request)))
  "Returns md5sum for a message object of type 'Current-request"
  "60f910a2f5cc14b982946ccdb2fea967")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Current-request>)))
  "Returns full string definition for message of type '<Current-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Current-request)))
  "Returns full string definition for message of type 'Current-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Current-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Current-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Current-request
))
;//! \htmlinclude Current-response.msg.html

(cl:defclass <Current-response> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (nodes
    :reader nodes
    :initarg :nodes
    :type cl:string
    :initform "")
   (current_time
    :reader current_time
    :initarg :current_time
    :type cl:float
    :initform 0.0)
   (running
    :reader running
    :initarg :running
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Current-response (<Current-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Current-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Current-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<Current-response> is deprecated: use performances-srv:Current-response instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <Current-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:ids-val is deprecated.  Use performances-srv:ids instead.")
  (ids m))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <Current-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:nodes-val is deprecated.  Use performances-srv:nodes instead.")
  (nodes m))

(cl:ensure-generic-function 'current_time-val :lambda-list '(m))
(cl:defmethod current_time-val ((m <Current-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:current_time-val is deprecated.  Use performances-srv:current_time instead.")
  (current_time m))

(cl:ensure-generic-function 'running-val :lambda-list '(m))
(cl:defmethod running-val ((m <Current-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:running-val is deprecated.  Use performances-srv:running instead.")
  (running m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Current-response>) ostream)
  "Serializes a message object of type '<Current-response>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodes))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'running) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Current-response>) istream)
  "Deserializes a message object of type '<Current-response>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'running) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Current-response>)))
  "Returns string type for a service object of type '<Current-response>"
  "performances/CurrentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Current-response)))
  "Returns string type for a service object of type 'Current-response"
  "performances/CurrentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Current-response>)))
  "Returns md5sum for a message object of type '<Current-response>"
  "60f910a2f5cc14b982946ccdb2fea967")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Current-response)))
  "Returns md5sum for a message object of type 'Current-response"
  "60f910a2f5cc14b982946ccdb2fea967")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Current-response>)))
  "Returns full string definition for message of type '<Current-response>"
  (cl:format cl:nil "string[] ids~%string nodes~%float32 current_time~%bool running~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Current-response)))
  "Returns full string definition for message of type 'Current-response"
  (cl:format cl:nil "string[] ids~%string nodes~%float32 current_time~%bool running~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Current-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'nodes))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Current-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Current-response
    (cl:cons ':ids (ids msg))
    (cl:cons ':nodes (nodes msg))
    (cl:cons ':current_time (current_time msg))
    (cl:cons ':running (running msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Current)))
  'Current-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Current)))
  'Current-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Current)))
  "Returns string type for a service object of type '<Current>"
  "performances/Current")