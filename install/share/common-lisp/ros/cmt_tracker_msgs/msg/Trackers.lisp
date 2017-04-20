; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-msg)


;//! \htmlinclude Trackers.msg.html

(cl:defclass <Trackers> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tracker_results
    :reader tracker_results
    :initarg :tracker_results
    :type (cl:vector cmt_tracker_msgs-msg:Tracker)
   :initform (cl:make-array 0 :element-type 'cmt_tracker_msgs-msg:Tracker :initial-element (cl:make-instance 'cmt_tracker_msgs-msg:Tracker))))
)

(cl:defclass Trackers (<Trackers>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trackers>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trackers)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-msg:<Trackers> is deprecated: use cmt_tracker_msgs-msg:Trackers instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Trackers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:header-val is deprecated.  Use cmt_tracker_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tracker_results-val :lambda-list '(m))
(cl:defmethod tracker_results-val ((m <Trackers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:tracker_results-val is deprecated.  Use cmt_tracker_msgs-msg:tracker_results instead.")
  (tracker_results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trackers>) ostream)
  "Serializes a message object of type '<Trackers>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracker_results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracker_results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trackers>) istream)
  "Deserializes a message object of type '<Trackers>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracker_results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracker_results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cmt_tracker_msgs-msg:Tracker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trackers>)))
  "Returns string type for a message object of type '<Trackers>"
  "cmt_tracker_msgs/Trackers")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trackers)))
  "Returns string type for a message object of type 'Trackers"
  "cmt_tracker_msgs/Trackers")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trackers>)))
  "Returns md5sum for a message object of type '<Trackers>"
  "8e27ab11ffdef13e75194ac70903ce07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trackers)))
  "Returns md5sum for a message object of type 'Trackers"
  "8e27ab11ffdef13e75194ac70903ce07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trackers>)))
  "Returns full string definition for message of type '<Trackers>"
  (cl:format cl:nil "Header header~%Tracker[] tracker_results~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmt_tracker_msgs/Tracker~%Header header~%Object object~%std_msgs/Int32 initial_points~%std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.~%std_msgs/Bool quality_results # Makes sense~%std_msgs/String tracker_name #This is needed.~%std_msgs/String recognized_name #This is assigned when it's known to have a name~%std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.~%std_msgs/Int32 before_demotion~%std_msgs/Bool validated #To check for merge fucntions.~%std_msgs/String google_best_guess~%================================================================================~%MSG: cmt_tracker_msgs/Object~%Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trackers)))
  "Returns full string definition for message of type 'Trackers"
  (cl:format cl:nil "Header header~%Tracker[] tracker_results~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmt_tracker_msgs/Tracker~%Header header~%Object object~%std_msgs/Int32 initial_points~%std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.~%std_msgs/Bool quality_results # Makes sense~%std_msgs/String tracker_name #This is needed.~%std_msgs/String recognized_name #This is assigned when it's known to have a name~%std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.~%std_msgs/Int32 before_demotion~%std_msgs/Bool validated #To check for merge fucntions.~%std_msgs/String google_best_guess~%================================================================================~%MSG: cmt_tracker_msgs/Object~%Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trackers>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracker_results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trackers>))
  "Converts a ROS message object to a list"
  (cl:list 'Trackers
    (cl:cons ':header (header msg))
    (cl:cons ':tracker_results (tracker_results msg))
))
