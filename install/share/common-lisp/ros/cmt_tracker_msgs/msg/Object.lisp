; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-msg)


;//! \htmlinclude Object.msg.html

(cl:defclass <Object> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object
    :reader object
    :initarg :object
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (id
    :reader id
    :initarg :id
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (obj_states
    :reader obj_states
    :initarg :obj_states
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (obj_accuracy
    :reader obj_accuracy
    :initarg :obj_accuracy
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (feature_point
    :reader feature_point
    :initarg :feature_point
    :type opencv_apps-msg:Point2DArray
    :initform (cl:make-instance 'opencv_apps-msg:Point2DArray))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (tool_used_for_detection
    :reader tool_used_for_detection
    :initarg :tool_used_for_detection
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass Object (<Object>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Object>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Object)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-msg:<Object> is deprecated: use cmt_tracker_msgs-msg:Object instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:header-val is deprecated.  Use cmt_tracker_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:object-val is deprecated.  Use cmt_tracker_msgs-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:id-val is deprecated.  Use cmt_tracker_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'obj_states-val :lambda-list '(m))
(cl:defmethod obj_states-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:obj_states-val is deprecated.  Use cmt_tracker_msgs-msg:obj_states instead.")
  (obj_states m))

(cl:ensure-generic-function 'obj_accuracy-val :lambda-list '(m))
(cl:defmethod obj_accuracy-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:obj_accuracy-val is deprecated.  Use cmt_tracker_msgs-msg:obj_accuracy instead.")
  (obj_accuracy m))

(cl:ensure-generic-function 'feature_point-val :lambda-list '(m))
(cl:defmethod feature_point-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:feature_point-val is deprecated.  Use cmt_tracker_msgs-msg:feature_point instead.")
  (feature_point m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:pose-val is deprecated.  Use cmt_tracker_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'tool_used_for_detection-val :lambda-list '(m))
(cl:defmethod tool_used_for_detection-val ((m <Object>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:tool_used_for_detection-val is deprecated.  Use cmt_tracker_msgs-msg:tool_used_for_detection instead.")
  (tool_used_for_detection m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Object>) ostream)
  "Serializes a message object of type '<Object>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obj_states) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obj_accuracy) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feature_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tool_used_for_detection) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Object>) istream)
  "Deserializes a message object of type '<Object>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obj_states) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obj_accuracy) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feature_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tool_used_for_detection) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Object>)))
  "Returns string type for a message object of type '<Object>"
  "cmt_tracker_msgs/Object")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Object)))
  "Returns string type for a message object of type 'Object"
  "cmt_tracker_msgs/Object")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Object>)))
  "Returns md5sum for a message object of type '<Object>"
  "232560d9417be3244e13955b54eafc19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Object)))
  "Returns md5sum for a message object of type 'Object"
  "232560d9417be3244e13955b54eafc19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Object>)))
  "Returns full string definition for message of type '<Object>"
  (cl:format cl:nil "Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Object)))
  "Returns full string definition for message of type 'Object"
  (cl:format cl:nil "Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Object>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obj_states))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obj_accuracy))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feature_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tool_used_for_detection))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Object>))
  "Converts a ROS message object to a list"
  (cl:list 'Object
    (cl:cons ':header (header msg))
    (cl:cons ':object (object msg))
    (cl:cons ':id (id msg))
    (cl:cons ':obj_states (obj_states msg))
    (cl:cons ':obj_accuracy (obj_accuracy msg))
    (cl:cons ':feature_point (feature_point msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':tool_used_for_detection (tool_used_for_detection msg))
))
