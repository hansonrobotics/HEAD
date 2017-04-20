; Auto-generated. Do not edit!


(cl:in-package manyears_msgs-msg)


;//! \htmlinclude SourceInfoWithCovariance.msg.html

(cl:defclass <SourceInfoWithCovariance> (roslisp-msg-protocol:ros-message)
  ((source_id
    :reader source_id
    :initarg :source_id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped))
   (source_probability
    :reader source_probability
    :initarg :source_probability
    :type cl:float
    :initform 0.0)
   (separation_data
    :reader separation_data
    :initarg :separation_data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (postfiltered_data
    :reader postfiltered_data
    :initarg :postfiltered_data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SourceInfoWithCovariance (<SourceInfoWithCovariance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SourceInfoWithCovariance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SourceInfoWithCovariance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manyears_msgs-msg:<SourceInfoWithCovariance> is deprecated: use manyears_msgs-msg:SourceInfoWithCovariance instead.")))

(cl:ensure-generic-function 'source_id-val :lambda-list '(m))
(cl:defmethod source_id-val ((m <SourceInfoWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:source_id-val is deprecated.  Use manyears_msgs-msg:source_id instead.")
  (source_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SourceInfoWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:pose-val is deprecated.  Use manyears_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'source_probability-val :lambda-list '(m))
(cl:defmethod source_probability-val ((m <SourceInfoWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:source_probability-val is deprecated.  Use manyears_msgs-msg:source_probability instead.")
  (source_probability m))

(cl:ensure-generic-function 'separation_data-val :lambda-list '(m))
(cl:defmethod separation_data-val ((m <SourceInfoWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:separation_data-val is deprecated.  Use manyears_msgs-msg:separation_data instead.")
  (separation_data m))

(cl:ensure-generic-function 'postfiltered_data-val :lambda-list '(m))
(cl:defmethod postfiltered_data-val ((m <SourceInfoWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:postfiltered_data-val is deprecated.  Use manyears_msgs-msg:postfiltered_data instead.")
  (postfiltered_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SourceInfoWithCovariance>) ostream)
  "Serializes a message object of type '<SourceInfoWithCovariance>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'source_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'source_probability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'separation_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'separation_data))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'postfiltered_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'postfiltered_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SourceInfoWithCovariance>) istream)
  "Deserializes a message object of type '<SourceInfoWithCovariance>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'source_probability) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'separation_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'separation_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'postfiltered_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'postfiltered_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SourceInfoWithCovariance>)))
  "Returns string type for a message object of type '<SourceInfoWithCovariance>"
  "manyears_msgs/SourceInfoWithCovariance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SourceInfoWithCovariance)))
  "Returns string type for a message object of type 'SourceInfoWithCovariance"
  "manyears_msgs/SourceInfoWithCovariance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SourceInfoWithCovariance>)))
  "Returns md5sum for a message object of type '<SourceInfoWithCovariance>"
  "0a935a273a55fd1e2e2cf21ca3fbcd68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SourceInfoWithCovariance)))
  "Returns md5sum for a message object of type 'SourceInfoWithCovariance"
  "0a935a273a55fd1e2e2cf21ca3fbcd68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SourceInfoWithCovariance>)))
  "Returns full string definition for message of type '<SourceInfoWithCovariance>"
  (cl:format cl:nil "# A source info message with a 3D-localized source with covariance.~%# Replaces the point + latitude and longitude info of a standard SourceInfo~%# message.~%~%uint32 source_id~%geometry_msgs/PoseWithCovarianceStamped pose~%float32 source_probability~%float32[] separation_data # Separation data (audio stream)~%float32[] postfiltered_data # Postfiltered data (audio stream)~%~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SourceInfoWithCovariance)))
  "Returns full string definition for message of type 'SourceInfoWithCovariance"
  (cl:format cl:nil "# A source info message with a 3D-localized source with covariance.~%# Replaces the point + latitude and longitude info of a standard SourceInfo~%# message.~%~%uint32 source_id~%geometry_msgs/PoseWithCovarianceStamped pose~%float32 source_probability~%float32[] separation_data # Separation data (audio stream)~%float32[] postfiltered_data # Postfiltered data (audio stream)~%~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SourceInfoWithCovariance>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'separation_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'postfiltered_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SourceInfoWithCovariance>))
  "Converts a ROS message object to a list"
  (cl:list 'SourceInfoWithCovariance
    (cl:cons ':source_id (source_id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':source_probability (source_probability msg))
    (cl:cons ':separation_data (separation_data msg))
    (cl:cons ':postfiltered_data (postfiltered_data msg))
))
