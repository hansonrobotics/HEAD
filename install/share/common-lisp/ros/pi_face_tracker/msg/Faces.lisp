; Auto-generated. Do not edit!


(cl:in-package pi_face_tracker-msg)


;//! \htmlinclude Faces.msg.html

(cl:defclass <Faces> (roslisp-msg-protocol:ros-message)
  ((faces
    :reader faces
    :initarg :faces
    :type (cl:vector pi_face_tracker-msg:Face)
   :initform (cl:make-array 0 :element-type 'pi_face_tracker-msg:Face :initial-element (cl:make-instance 'pi_face_tracker-msg:Face))))
)

(cl:defclass Faces (<Faces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Faces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Faces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pi_face_tracker-msg:<Faces> is deprecated: use pi_face_tracker-msg:Faces instead.")))

(cl:ensure-generic-function 'faces-val :lambda-list '(m))
(cl:defmethod faces-val ((m <Faces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pi_face_tracker-msg:faces-val is deprecated.  Use pi_face_tracker-msg:faces instead.")
  (faces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Faces>) ostream)
  "Serializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faces))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faces))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Faces>) istream)
  "Deserializes a message object of type '<Faces>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faces) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faces)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pi_face_tracker-msg:Face))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Faces>)))
  "Returns string type for a message object of type '<Faces>"
  "pi_face_tracker/Faces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Faces)))
  "Returns string type for a message object of type 'Faces"
  "pi_face_tracker/Faces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Faces>)))
  "Returns md5sum for a message object of type '<Faces>"
  "2941363c8187ea20fb8eed7897d2a68a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Faces)))
  "Returns md5sum for a message object of type 'Faces"
  "2941363c8187ea20fb8eed7897d2a68a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Faces>)))
  "Returns full string definition for message of type '<Faces>"
  (cl:format cl:nil "# Multiple faces~%Face[] faces~%================================================================================~%MSG: pi_face_tracker/Face~%# Face in 3D space~%int32 id~%geometry_msgs/Point point~%float32 attention~%# Emotion~%int32 emotion_value~%string emotion_id~%#Temporary~%bool temp_id~%bool recognized~%string recognized_as~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Faces)))
  "Returns full string definition for message of type 'Faces"
  (cl:format cl:nil "# Multiple faces~%Face[] faces~%================================================================================~%MSG: pi_face_tracker/Face~%# Face in 3D space~%int32 id~%geometry_msgs/Point point~%float32 attention~%# Emotion~%int32 emotion_value~%string emotion_id~%#Temporary~%bool temp_id~%bool recognized~%string recognized_as~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Faces>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faces) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Faces>))
  "Converts a ROS message object to a list"
  (cl:list 'Faces
    (cl:cons ':faces (faces msg))
))
