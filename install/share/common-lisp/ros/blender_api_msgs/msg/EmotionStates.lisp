; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude EmotionStates.msg.html

(cl:defclass <EmotionStates> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector blender_api_msgs-msg:EmotionState)
   :initform (cl:make-array 0 :element-type 'blender_api_msgs-msg:EmotionState :initial-element (cl:make-instance 'blender_api_msgs-msg:EmotionState))))
)

(cl:defclass EmotionStates (<EmotionStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<EmotionStates> is deprecated: use blender_api_msgs-msg:EmotionStates instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <EmotionStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:data-val is deprecated.  Use blender_api_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionStates>) ostream)
  "Serializes a message object of type '<EmotionStates>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionStates>) istream)
  "Deserializes a message object of type '<EmotionStates>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'blender_api_msgs-msg:EmotionState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionStates>)))
  "Returns string type for a message object of type '<EmotionStates>"
  "blender_api_msgs/EmotionStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionStates)))
  "Returns string type for a message object of type 'EmotionStates"
  "blender_api_msgs/EmotionStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionStates>)))
  "Returns md5sum for a message object of type '<EmotionStates>"
  "70bbe0027e7f7d2833349094898bfcec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionStates)))
  "Returns md5sum for a message object of type 'EmotionStates"
  "70bbe0027e7f7d2833349094898bfcec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionStates>)))
  "Returns full string definition for message of type '<EmotionStates>"
  (cl:format cl:nil "EmotionState[] data~%~%================================================================================~%MSG: blender_api_msgs/EmotionState~%string name~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionStates)))
  "Returns full string definition for message of type 'EmotionStates"
  (cl:format cl:nil "EmotionState[] data~%~%================================================================================~%MSG: blender_api_msgs/EmotionState~%string name~%float32 magnitude~%duration duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionStates>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionStates>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionStates
    (cl:cons ':data (data msg))
))
