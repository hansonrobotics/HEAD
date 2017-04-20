; Auto-generated. Do not edit!


(cl:in-package basic_head_api-srv)


;//! \htmlinclude ValidFaceExprs-request.msg.html

(cl:defclass <ValidFaceExprs-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ValidFaceExprs-request (<ValidFaceExprs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidFaceExprs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidFaceExprs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-srv:<ValidFaceExprs-request> is deprecated: use basic_head_api-srv:ValidFaceExprs-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidFaceExprs-request>) ostream)
  "Serializes a message object of type '<ValidFaceExprs-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidFaceExprs-request>) istream)
  "Deserializes a message object of type '<ValidFaceExprs-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidFaceExprs-request>)))
  "Returns string type for a service object of type '<ValidFaceExprs-request>"
  "basic_head_api/ValidFaceExprsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidFaceExprs-request)))
  "Returns string type for a service object of type 'ValidFaceExprs-request"
  "basic_head_api/ValidFaceExprsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidFaceExprs-request>)))
  "Returns md5sum for a message object of type '<ValidFaceExprs-request>"
  "961110ff326afbad135f5b318e4e8821")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidFaceExprs-request)))
  "Returns md5sum for a message object of type 'ValidFaceExprs-request"
  "961110ff326afbad135f5b318e4e8821")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidFaceExprs-request>)))
  "Returns full string definition for message of type '<ValidFaceExprs-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidFaceExprs-request)))
  "Returns full string definition for message of type 'ValidFaceExprs-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidFaceExprs-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidFaceExprs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidFaceExprs-request
))
;//! \htmlinclude ValidFaceExprs-response.msg.html

(cl:defclass <ValidFaceExprs-response> (roslisp-msg-protocol:ros-message)
  ((exprnames
    :reader exprnames
    :initarg :exprnames
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ValidFaceExprs-response (<ValidFaceExprs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidFaceExprs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidFaceExprs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-srv:<ValidFaceExprs-response> is deprecated: use basic_head_api-srv:ValidFaceExprs-response instead.")))

(cl:ensure-generic-function 'exprnames-val :lambda-list '(m))
(cl:defmethod exprnames-val ((m <ValidFaceExprs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-srv:exprnames-val is deprecated.  Use basic_head_api-srv:exprnames instead.")
  (exprnames m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidFaceExprs-response>) ostream)
  "Serializes a message object of type '<ValidFaceExprs-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exprnames))))
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
   (cl:slot-value msg 'exprnames))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidFaceExprs-response>) istream)
  "Deserializes a message object of type '<ValidFaceExprs-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exprnames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exprnames)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidFaceExprs-response>)))
  "Returns string type for a service object of type '<ValidFaceExprs-response>"
  "basic_head_api/ValidFaceExprsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidFaceExprs-response)))
  "Returns string type for a service object of type 'ValidFaceExprs-response"
  "basic_head_api/ValidFaceExprsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidFaceExprs-response>)))
  "Returns md5sum for a message object of type '<ValidFaceExprs-response>"
  "961110ff326afbad135f5b318e4e8821")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidFaceExprs-response)))
  "Returns md5sum for a message object of type 'ValidFaceExprs-response"
  "961110ff326afbad135f5b318e4e8821")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidFaceExprs-response>)))
  "Returns full string definition for message of type '<ValidFaceExprs-response>"
  (cl:format cl:nil "string[] exprnames~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidFaceExprs-response)))
  "Returns full string definition for message of type 'ValidFaceExprs-response"
  (cl:format cl:nil "string[] exprnames~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidFaceExprs-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exprnames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidFaceExprs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidFaceExprs-response
    (cl:cons ':exprnames (exprnames msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ValidFaceExprs)))
  'ValidFaceExprs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ValidFaceExprs)))
  'ValidFaceExprs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidFaceExprs)))
  "Returns string type for a service object of type '<ValidFaceExprs>"
  "basic_head_api/ValidFaceExprs")