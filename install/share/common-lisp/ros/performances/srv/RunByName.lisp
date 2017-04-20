; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude RunByName-request.msg.html

(cl:defclass <RunByName-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass RunByName-request (<RunByName-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunByName-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunByName-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<RunByName-request> is deprecated: use performances-srv:RunByName-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RunByName-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:id-val is deprecated.  Use performances-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunByName-request>) ostream)
  "Serializes a message object of type '<RunByName-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunByName-request>) istream)
  "Deserializes a message object of type '<RunByName-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunByName-request>)))
  "Returns string type for a service object of type '<RunByName-request>"
  "performances/RunByNameRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunByName-request)))
  "Returns string type for a service object of type 'RunByName-request"
  "performances/RunByNameRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunByName-request>)))
  "Returns md5sum for a message object of type '<RunByName-request>"
  "4dc5c1ae2e494627977ff6a6eeb914ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunByName-request)))
  "Returns md5sum for a message object of type 'RunByName-request"
  "4dc5c1ae2e494627977ff6a6eeb914ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunByName-request>)))
  "Returns full string definition for message of type '<RunByName-request>"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunByName-request)))
  "Returns full string definition for message of type 'RunByName-request"
  (cl:format cl:nil "string id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunByName-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunByName-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunByName-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude RunByName-response.msg.html

(cl:defclass <RunByName-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RunByName-response (<RunByName-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunByName-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunByName-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<RunByName-response> is deprecated: use performances-srv:RunByName-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RunByName-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunByName-response>) ostream)
  "Serializes a message object of type '<RunByName-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunByName-response>) istream)
  "Deserializes a message object of type '<RunByName-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunByName-response>)))
  "Returns string type for a service object of type '<RunByName-response>"
  "performances/RunByNameResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunByName-response)))
  "Returns string type for a service object of type 'RunByName-response"
  "performances/RunByNameResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunByName-response>)))
  "Returns md5sum for a message object of type '<RunByName-response>"
  "4dc5c1ae2e494627977ff6a6eeb914ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunByName-response)))
  "Returns md5sum for a message object of type 'RunByName-response"
  "4dc5c1ae2e494627977ff6a6eeb914ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunByName-response>)))
  "Returns full string definition for message of type '<RunByName-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunByName-response)))
  "Returns full string definition for message of type 'RunByName-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunByName-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunByName-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunByName-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunByName)))
  'RunByName-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunByName)))
  'RunByName-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunByName)))
  "Returns string type for a service object of type '<RunByName>"
  "performances/RunByName")