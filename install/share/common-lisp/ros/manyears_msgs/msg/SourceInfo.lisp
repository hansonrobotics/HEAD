; Auto-generated. Do not edit!


(cl:in-package manyears_msgs-msg)


;//! \htmlinclude SourceInfo.msg.html

(cl:defclass <SourceInfo> (roslisp-msg-protocol:ros-message)
  ((source_id
    :reader source_id
    :initarg :source_id
    :type cl:integer
    :initform 0)
   (source_pos
    :reader source_pos
    :initarg :source_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
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

(cl:defclass SourceInfo (<SourceInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SourceInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SourceInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manyears_msgs-msg:<SourceInfo> is deprecated: use manyears_msgs-msg:SourceInfo instead.")))

(cl:ensure-generic-function 'source_id-val :lambda-list '(m))
(cl:defmethod source_id-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:source_id-val is deprecated.  Use manyears_msgs-msg:source_id instead.")
  (source_id m))

(cl:ensure-generic-function 'source_pos-val :lambda-list '(m))
(cl:defmethod source_pos-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:source_pos-val is deprecated.  Use manyears_msgs-msg:source_pos instead.")
  (source_pos m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:longitude-val is deprecated.  Use manyears_msgs-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:latitude-val is deprecated.  Use manyears_msgs-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'source_probability-val :lambda-list '(m))
(cl:defmethod source_probability-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:source_probability-val is deprecated.  Use manyears_msgs-msg:source_probability instead.")
  (source_probability m))

(cl:ensure-generic-function 'separation_data-val :lambda-list '(m))
(cl:defmethod separation_data-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:separation_data-val is deprecated.  Use manyears_msgs-msg:separation_data instead.")
  (separation_data m))

(cl:ensure-generic-function 'postfiltered_data-val :lambda-list '(m))
(cl:defmethod postfiltered_data-val ((m <SourceInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_msgs-msg:postfiltered_data-val is deprecated.  Use manyears_msgs-msg:postfiltered_data instead.")
  (postfiltered_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SourceInfo>) ostream)
  "Serializes a message object of type '<SourceInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'source_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'source_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source_pos) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SourceInfo>) istream)
  "Deserializes a message object of type '<SourceInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'source_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source_pos) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SourceInfo>)))
  "Returns string type for a message object of type '<SourceInfo>"
  "manyears_msgs/SourceInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SourceInfo)))
  "Returns string type for a message object of type 'SourceInfo"
  "manyears_msgs/SourceInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SourceInfo>)))
  "Returns md5sum for a message object of type '<SourceInfo>"
  "94f909a2797d50c0ca7dbb707ce66302")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SourceInfo)))
  "Returns md5sum for a message object of type 'SourceInfo"
  "94f909a2797d50c0ca7dbb707ce66302")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SourceInfo>)))
  "Returns full string definition for message of type '<SourceInfo>"
  (cl:format cl:nil "#Tracked source information~%uint32 source_id~%geometry_msgs/Point source_pos~%float32 longitude   # In degrees~%float32 latitude    # In degrees ~%float32 source_probability~%float32[] separation_data # Separation data (audio stream)~%float32[] postfiltered_data # Postfiltered data (audio stream)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SourceInfo)))
  "Returns full string definition for message of type 'SourceInfo"
  (cl:format cl:nil "#Tracked source information~%uint32 source_id~%geometry_msgs/Point source_pos~%float32 longitude   # In degrees~%float32 latitude    # In degrees ~%float32 source_probability~%float32[] separation_data # Separation data (audio stream)~%float32[] postfiltered_data # Postfiltered data (audio stream)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SourceInfo>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source_pos))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'separation_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'postfiltered_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SourceInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SourceInfo
    (cl:cons ':source_id (source_id msg))
    (cl:cons ':source_pos (source_pos msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':source_probability (source_probability msg))
    (cl:cons ':separation_data (separation_data msg))
    (cl:cons ':postfiltered_data (postfiltered_data msg))
))
