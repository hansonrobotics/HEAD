; Auto-generated. Do not edit!


(cl:in-package cmt_tracker_msgs-msg)


;//! \htmlinclude Tracker.msg.html

(cl:defclass <Tracker> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object
    :reader object
    :initarg :object
    :type cmt_tracker_msgs-msg:Object
    :initform (cl:make-instance 'cmt_tracker_msgs-msg:Object))
   (initial_points
    :reader initial_points
    :initarg :initial_points
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (active_points
    :reader active_points
    :initarg :active_points
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (quality_results
    :reader quality_results
    :initarg :quality_results
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (tracker_name
    :reader tracker_name
    :initarg :tracker_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (recognized_name
    :reader recognized_name
    :initarg :recognized_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (recognized
    :reader recognized
    :initarg :recognized
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (before_demotion
    :reader before_demotion
    :initarg :before_demotion
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (validated
    :reader validated
    :initarg :validated
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (google_best_guess
    :reader google_best_guess
    :initarg :google_best_guess
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass Tracker (<Tracker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tracker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tracker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmt_tracker_msgs-msg:<Tracker> is deprecated: use cmt_tracker_msgs-msg:Tracker instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:header-val is deprecated.  Use cmt_tracker_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:object-val is deprecated.  Use cmt_tracker_msgs-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'initial_points-val :lambda-list '(m))
(cl:defmethod initial_points-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:initial_points-val is deprecated.  Use cmt_tracker_msgs-msg:initial_points instead.")
  (initial_points m))

(cl:ensure-generic-function 'active_points-val :lambda-list '(m))
(cl:defmethod active_points-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:active_points-val is deprecated.  Use cmt_tracker_msgs-msg:active_points instead.")
  (active_points m))

(cl:ensure-generic-function 'quality_results-val :lambda-list '(m))
(cl:defmethod quality_results-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:quality_results-val is deprecated.  Use cmt_tracker_msgs-msg:quality_results instead.")
  (quality_results m))

(cl:ensure-generic-function 'tracker_name-val :lambda-list '(m))
(cl:defmethod tracker_name-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:tracker_name-val is deprecated.  Use cmt_tracker_msgs-msg:tracker_name instead.")
  (tracker_name m))

(cl:ensure-generic-function 'recognized_name-val :lambda-list '(m))
(cl:defmethod recognized_name-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:recognized_name-val is deprecated.  Use cmt_tracker_msgs-msg:recognized_name instead.")
  (recognized_name m))

(cl:ensure-generic-function 'recognized-val :lambda-list '(m))
(cl:defmethod recognized-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:recognized-val is deprecated.  Use cmt_tracker_msgs-msg:recognized instead.")
  (recognized m))

(cl:ensure-generic-function 'before_demotion-val :lambda-list '(m))
(cl:defmethod before_demotion-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:before_demotion-val is deprecated.  Use cmt_tracker_msgs-msg:before_demotion instead.")
  (before_demotion m))

(cl:ensure-generic-function 'validated-val :lambda-list '(m))
(cl:defmethod validated-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:validated-val is deprecated.  Use cmt_tracker_msgs-msg:validated instead.")
  (validated m))

(cl:ensure-generic-function 'google_best_guess-val :lambda-list '(m))
(cl:defmethod google_best_guess-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmt_tracker_msgs-msg:google_best_guess-val is deprecated.  Use cmt_tracker_msgs-msg:google_best_guess instead.")
  (google_best_guess m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tracker>) ostream)
  "Serializes a message object of type '<Tracker>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initial_points) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'active_points) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quality_results) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tracker_name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'recognized_name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'recognized) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'before_demotion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'validated) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'google_best_guess) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tracker>) istream)
  "Deserializes a message object of type '<Tracker>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initial_points) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'active_points) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quality_results) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tracker_name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'recognized_name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'recognized) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'before_demotion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'validated) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'google_best_guess) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tracker>)))
  "Returns string type for a message object of type '<Tracker>"
  "cmt_tracker_msgs/Tracker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tracker)))
  "Returns string type for a message object of type 'Tracker"
  "cmt_tracker_msgs/Tracker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tracker>)))
  "Returns md5sum for a message object of type '<Tracker>"
  "a6a14d053d2c664581f299a7e8a149db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tracker)))
  "Returns md5sum for a message object of type 'Tracker"
  "a6a14d053d2c664581f299a7e8a149db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tracker>)))
  "Returns full string definition for message of type '<Tracker>"
  (cl:format cl:nil "Header header~%Object object~%std_msgs/Int32 initial_points~%std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.~%std_msgs/Bool quality_results # Makes sense~%std_msgs/String tracker_name #This is needed.~%std_msgs/String recognized_name #This is assigned when it's known to have a name~%std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.~%std_msgs/Int32 before_demotion~%std_msgs/Bool validated #To check for merge fucntions.~%std_msgs/String google_best_guess~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmt_tracker_msgs/Object~%Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tracker)))
  "Returns full string definition for message of type 'Tracker"
  (cl:format cl:nil "Header header~%Object object~%std_msgs/Int32 initial_points~%std_msgs/Int32 active_points #Moving avaerage to destroy as it wouldn't make sense just to remove.~%std_msgs/Bool quality_results # Makes sense~%std_msgs/String tracker_name #This is needed.~%std_msgs/String recognized_name #This is assigned when it's known to have a name~%std_msgs/Bool recognized # This is updated after tracker_name is updated from the recognizer function.~%std_msgs/Int32 before_demotion~%std_msgs/Bool validated #To check for merge fucntions.~%std_msgs/String google_best_guess~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmt_tracker_msgs/Object~%Header header~%sensor_msgs/RegionOfInterest object~%std_msgs/Int32 id~%std_msgs/String obj_states~%std_msgs/Float64 obj_accuracy~%opencv_apps/Point2DArray feature_point~%geometry_msgs/Pose pose~%std_msgs/String tool_used_for_detection~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: opencv_apps/Point2DArray~%Point2D[] points~%~%================================================================================~%MSG: opencv_apps/Point2D~%float64 x~%float64 y~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tracker>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initial_points))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'active_points))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quality_results))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tracker_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'recognized_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'recognized))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'before_demotion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'validated))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'google_best_guess))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tracker>))
  "Converts a ROS message object to a list"
  (cl:list 'Tracker
    (cl:cons ':header (header msg))
    (cl:cons ':object (object msg))
    (cl:cons ':initial_points (initial_points msg))
    (cl:cons ':active_points (active_points msg))
    (cl:cons ':quality_results (quality_results msg))
    (cl:cons ':tracker_name (tracker_name msg))
    (cl:cons ':recognized_name (recognized_name msg))
    (cl:cons ':recognized (recognized msg))
    (cl:cons ':before_demotion (before_demotion msg))
    (cl:cons ':validated (validated msg))
    (cl:cons ':google_best_guess (google_best_guess msg))
))
