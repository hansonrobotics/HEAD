; Auto-generated. Do not edit!


(cl:in-package ros_nmpt_saliency-msg)


;//! \htmlinclude targets.msg.html

(cl:defclass <targets> (roslisp-msg-protocol:ros-message)
  ((positions
    :reader positions
    :initarg :positions
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (degree
    :reader degree
    :initarg :degree
    :type cl:float
    :initform 0.0))
)

(cl:defclass targets (<targets>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <targets>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'targets)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_nmpt_saliency-msg:<targets> is deprecated: use ros_nmpt_saliency-msg:targets instead.")))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <targets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_nmpt_saliency-msg:positions-val is deprecated.  Use ros_nmpt_saliency-msg:positions instead.")
  (positions m))

(cl:ensure-generic-function 'degree-val :lambda-list '(m))
(cl:defmethod degree-val ((m <targets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_nmpt_saliency-msg:degree-val is deprecated.  Use ros_nmpt_saliency-msg:degree instead.")
  (degree m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <targets>) ostream)
  "Serializes a message object of type '<targets>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'positions))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'degree))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <targets>) istream)
  "Deserializes a message object of type '<targets>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'degree) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<targets>)))
  "Returns string type for a message object of type '<targets>"
  "ros_nmpt_saliency/targets")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'targets)))
  "Returns string type for a message object of type 'targets"
  "ros_nmpt_saliency/targets")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<targets>)))
  "Returns md5sum for a message object of type '<targets>"
  "eaf8d5353bd6068bc7a7a3420ba17cf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'targets)))
  "Returns md5sum for a message object of type 'targets"
  "eaf8d5353bd6068bc7a7a3420ba17cf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<targets>)))
  "Returns full string definition for message of type '<targets>"
  (cl:format cl:nil "geometry_msgs/Point[] positions~%float32 degree~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'targets)))
  "Returns full string definition for message of type 'targets"
  (cl:format cl:nil "geometry_msgs/Point[] positions~%float32 degree~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <targets>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <targets>))
  "Converts a ROS message object to a list"
  (cl:list 'targets
    (cl:cons ':positions (positions msg))
    (cl:cons ':degree (degree msg))
))
