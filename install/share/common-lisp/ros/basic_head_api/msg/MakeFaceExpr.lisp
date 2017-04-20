; Auto-generated. Do not edit!


(cl:in-package basic_head_api-msg)


;//! \htmlinclude MakeFaceExpr.msg.html

(cl:defclass <MakeFaceExpr> (roslisp-msg-protocol:ros-message)
  ((exprname
    :reader exprname
    :initarg :exprname
    :type cl:string
    :initform "")
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:float
    :initform 0.0))
)

(cl:defclass MakeFaceExpr (<MakeFaceExpr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MakeFaceExpr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MakeFaceExpr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_head_api-msg:<MakeFaceExpr> is deprecated: use basic_head_api-msg:MakeFaceExpr instead.")))

(cl:ensure-generic-function 'exprname-val :lambda-list '(m))
(cl:defmethod exprname-val ((m <MakeFaceExpr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:exprname-val is deprecated.  Use basic_head_api-msg:exprname instead.")
  (exprname m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <MakeFaceExpr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_head_api-msg:intensity-val is deprecated.  Use basic_head_api-msg:intensity instead.")
  (intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MakeFaceExpr>) ostream)
  "Serializes a message object of type '<MakeFaceExpr>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'exprname))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'exprname))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'intensity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MakeFaceExpr>) istream)
  "Deserializes a message object of type '<MakeFaceExpr>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exprname) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'exprname) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'intensity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MakeFaceExpr>)))
  "Returns string type for a message object of type '<MakeFaceExpr>"
  "basic_head_api/MakeFaceExpr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MakeFaceExpr)))
  "Returns string type for a message object of type 'MakeFaceExpr"
  "basic_head_api/MakeFaceExpr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MakeFaceExpr>)))
  "Returns md5sum for a message object of type '<MakeFaceExpr>"
  "23f4cd70e7885a30cd4d3e8e82073188")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MakeFaceExpr)))
  "Returns md5sum for a message object of type 'MakeFaceExpr"
  "23f4cd70e7885a30cd4d3e8e82073188")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MakeFaceExpr>)))
  "Returns full string definition for message of type '<MakeFaceExpr>"
  (cl:format cl:nil "string exprname~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MakeFaceExpr)))
  "Returns full string definition for message of type 'MakeFaceExpr"
  (cl:format cl:nil "string exprname~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MakeFaceExpr>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'exprname))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MakeFaceExpr>))
  "Converts a ROS message object to a list"
  (cl:list 'MakeFaceExpr
    (cl:cons ':exprname (exprname msg))
    (cl:cons ':intensity (intensity msg))
))
