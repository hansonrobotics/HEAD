; Auto-generated. Do not edit!


(cl:in-package performances-srv)


;//! \htmlinclude LoadNodes-request.msg.html

(cl:defclass <LoadNodes-request> (roslisp-msg-protocol:ros-message)
  ((nodes
    :reader nodes
    :initarg :nodes
    :type cl:string
    :initform ""))
)

(cl:defclass LoadNodes-request (<LoadNodes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadNodes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadNodes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<LoadNodes-request> is deprecated: use performances-srv:LoadNodes-request instead.")))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <LoadNodes-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:nodes-val is deprecated.  Use performances-srv:nodes instead.")
  (nodes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadNodes-request>) ostream)
  "Serializes a message object of type '<LoadNodes-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nodes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadNodes-request>) istream)
  "Deserializes a message object of type '<LoadNodes-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodes) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nodes) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadNodes-request>)))
  "Returns string type for a service object of type '<LoadNodes-request>"
  "performances/LoadNodesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadNodes-request)))
  "Returns string type for a service object of type 'LoadNodes-request"
  "performances/LoadNodesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadNodes-request>)))
  "Returns md5sum for a message object of type '<LoadNodes-request>"
  "01860f8dc9bc013e894571b93a50f642")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadNodes-request)))
  "Returns md5sum for a message object of type 'LoadNodes-request"
  "01860f8dc9bc013e894571b93a50f642")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadNodes-request>)))
  "Returns full string definition for message of type '<LoadNodes-request>"
  (cl:format cl:nil "string nodes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadNodes-request)))
  "Returns full string definition for message of type 'LoadNodes-request"
  (cl:format cl:nil "string nodes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadNodes-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'nodes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadNodes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadNodes-request
    (cl:cons ':nodes (nodes msg))
))
;//! \htmlinclude LoadNodes-response.msg.html

(cl:defclass <LoadNodes-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadNodes-response (<LoadNodes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadNodes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadNodes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name performances-srv:<LoadNodes-response> is deprecated: use performances-srv:LoadNodes-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadNodes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader performances-srv:success-val is deprecated.  Use performances-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadNodes-response>) ostream)
  "Serializes a message object of type '<LoadNodes-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadNodes-response>) istream)
  "Deserializes a message object of type '<LoadNodes-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadNodes-response>)))
  "Returns string type for a service object of type '<LoadNodes-response>"
  "performances/LoadNodesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadNodes-response)))
  "Returns string type for a service object of type 'LoadNodes-response"
  "performances/LoadNodesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadNodes-response>)))
  "Returns md5sum for a message object of type '<LoadNodes-response>"
  "01860f8dc9bc013e894571b93a50f642")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadNodes-response)))
  "Returns md5sum for a message object of type 'LoadNodes-response"
  "01860f8dc9bc013e894571b93a50f642")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadNodes-response>)))
  "Returns full string definition for message of type '<LoadNodes-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadNodes-response)))
  "Returns full string definition for message of type 'LoadNodes-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadNodes-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadNodes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadNodes-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadNodes)))
  'LoadNodes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadNodes)))
  'LoadNodes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadNodes)))
  "Returns string type for a service object of type '<LoadNodes>"
  "performances/LoadNodes")