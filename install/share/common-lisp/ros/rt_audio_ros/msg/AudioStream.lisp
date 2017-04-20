; Auto-generated. Do not edit!


(cl:in-package rt_audio_ros-msg)


;//! \htmlinclude AudioStream.msg.html

(cl:defclass <AudioStream> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (encoding
    :reader encoding
    :initarg :encoding
    :type cl:fixnum
    :initform 0)
   (is_bigendian
    :reader is_bigendian
    :initarg :is_bigendian
    :type cl:fixnum
    :initform 0)
   (channels
    :reader channels
    :initarg :channels
    :type cl:fixnum
    :initform 0)
   (sample_rate
    :reader sample_rate
    :initarg :sample_rate
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass AudioStream (<AudioStream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AudioStream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AudioStream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rt_audio_ros-msg:<AudioStream> is deprecated: use rt_audio_ros-msg:AudioStream instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:header-val is deprecated.  Use rt_audio_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'encoding-val :lambda-list '(m))
(cl:defmethod encoding-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:encoding-val is deprecated.  Use rt_audio_ros-msg:encoding instead.")
  (encoding m))

(cl:ensure-generic-function 'is_bigendian-val :lambda-list '(m))
(cl:defmethod is_bigendian-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:is_bigendian-val is deprecated.  Use rt_audio_ros-msg:is_bigendian instead.")
  (is_bigendian m))

(cl:ensure-generic-function 'channels-val :lambda-list '(m))
(cl:defmethod channels-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:channels-val is deprecated.  Use rt_audio_ros-msg:channels instead.")
  (channels m))

(cl:ensure-generic-function 'sample_rate-val :lambda-list '(m))
(cl:defmethod sample_rate-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:sample_rate-val is deprecated.  Use rt_audio_ros-msg:sample_rate instead.")
  (sample_rate m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rt_audio_ros-msg:data-val is deprecated.  Use rt_audio_ros-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AudioStream>)))
    "Constants for message type '<AudioStream>"
  '((:SINT_8_PCM . 0)
    (:UINT_8_PCM . 1)
    (:SINT_16_PCM . 2)
    (:SINT_24_PCM . 3)
    (:SINT_32_PCM . 4)
    (:FLOAT_32 . 5)
    (:FLOAT_64 . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AudioStream)))
    "Constants for message type 'AudioStream"
  '((:SINT_8_PCM . 0)
    (:UINT_8_PCM . 1)
    (:SINT_16_PCM . 2)
    (:SINT_24_PCM . 3)
    (:SINT_32_PCM . 4)
    (:FLOAT_32 . 5)
    (:FLOAT_64 . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AudioStream>) ostream)
  "Serializes a message object of type '<AudioStream>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoding)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_bigendian)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channels)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_rate)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AudioStream>) istream)
  "Deserializes a message object of type '<AudioStream>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoding)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_bigendian)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channels)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_rate)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AudioStream>)))
  "Returns string type for a message object of type '<AudioStream>"
  "rt_audio_ros/AudioStream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AudioStream)))
  "Returns string type for a message object of type 'AudioStream"
  "rt_audio_ros/AudioStream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AudioStream>)))
  "Returns md5sum for a message object of type '<AudioStream>"
  "9370a1d8dc6e6a70477f76a5472091f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AudioStream)))
  "Returns md5sum for a message object of type 'AudioStream"
  "9370a1d8dc6e6a70477f76a5472091f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AudioStream>)))
  "Returns full string definition for message of type '<AudioStream>"
  (cl:format cl:nil "Header header~%~%uint8 encoding~%~%uint8 SINT_8_PCM=0~%uint8 UINT_8_PCM=1~%uint8 SINT_16_PCM=2~%uint8 SINT_24_PCM=3~%uint8 SINT_32_PCM=4~%uint8 FLOAT_32=5~%uint8 FLOAT_64=6~%~%uint8  is_bigendian~%uint8  channels~%uint32 sample_rate~%~%# interleaved channels~%uint8[]  data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AudioStream)))
  "Returns full string definition for message of type 'AudioStream"
  (cl:format cl:nil "Header header~%~%uint8 encoding~%~%uint8 SINT_8_PCM=0~%uint8 UINT_8_PCM=1~%uint8 SINT_16_PCM=2~%uint8 SINT_24_PCM=3~%uint8 SINT_32_PCM=4~%uint8 FLOAT_32=5~%uint8 FLOAT_64=6~%~%uint8  is_bigendian~%uint8  channels~%uint32 sample_rate~%~%# interleaved channels~%uint8[]  data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AudioStream>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AudioStream>))
  "Converts a ROS message object to a list"
  (cl:list 'AudioStream
    (cl:cons ':header (header msg))
    (cl:cons ':encoding (encoding msg))
    (cl:cons ':is_bigendian (is_bigendian msg))
    (cl:cons ':channels (channels msg))
    (cl:cons ':sample_rate (sample_rate msg))
    (cl:cons ':data (data msg))
))
