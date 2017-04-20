; Auto-generated. Do not edit!


(cl:in-package pau2motors-msg)


;//! \htmlinclude pau.msg.html

(cl:defclass <pau> (roslisp-msg-protocol:ros-message)
  ((m_headRotation
    :reader m_headRotation
    :initarg :m_headRotation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (m_headTranslation
    :reader m_headTranslation
    :initarg :m_headTranslation
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (m_neckRotation
    :reader m_neckRotation
    :initarg :m_neckRotation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (m_eyeGazeLeftPitch
    :reader m_eyeGazeLeftPitch
    :initarg :m_eyeGazeLeftPitch
    :type cl:float
    :initform 0.0)
   (m_eyeGazeLeftYaw
    :reader m_eyeGazeLeftYaw
    :initarg :m_eyeGazeLeftYaw
    :type cl:float
    :initform 0.0)
   (m_eyeGazeRightPitch
    :reader m_eyeGazeRightPitch
    :initarg :m_eyeGazeRightPitch
    :type cl:float
    :initform 0.0)
   (m_eyeGazeRightYaw
    :reader m_eyeGazeRightYaw
    :initarg :m_eyeGazeRightYaw
    :type cl:float
    :initform 0.0)
   (m_coeffs
    :reader m_coeffs
    :initarg :m_coeffs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (m_shapekeys
    :reader m_shapekeys
    :initarg :m_shapekeys
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass pau (<pau>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pau>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pau)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pau2motors-msg:<pau> is deprecated: use pau2motors-msg:pau instead.")))

(cl:ensure-generic-function 'm_headRotation-val :lambda-list '(m))
(cl:defmethod m_headRotation-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_headRotation-val is deprecated.  Use pau2motors-msg:m_headRotation instead.")
  (m_headRotation m))

(cl:ensure-generic-function 'm_headTranslation-val :lambda-list '(m))
(cl:defmethod m_headTranslation-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_headTranslation-val is deprecated.  Use pau2motors-msg:m_headTranslation instead.")
  (m_headTranslation m))

(cl:ensure-generic-function 'm_neckRotation-val :lambda-list '(m))
(cl:defmethod m_neckRotation-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_neckRotation-val is deprecated.  Use pau2motors-msg:m_neckRotation instead.")
  (m_neckRotation m))

(cl:ensure-generic-function 'm_eyeGazeLeftPitch-val :lambda-list '(m))
(cl:defmethod m_eyeGazeLeftPitch-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_eyeGazeLeftPitch-val is deprecated.  Use pau2motors-msg:m_eyeGazeLeftPitch instead.")
  (m_eyeGazeLeftPitch m))

(cl:ensure-generic-function 'm_eyeGazeLeftYaw-val :lambda-list '(m))
(cl:defmethod m_eyeGazeLeftYaw-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_eyeGazeLeftYaw-val is deprecated.  Use pau2motors-msg:m_eyeGazeLeftYaw instead.")
  (m_eyeGazeLeftYaw m))

(cl:ensure-generic-function 'm_eyeGazeRightPitch-val :lambda-list '(m))
(cl:defmethod m_eyeGazeRightPitch-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_eyeGazeRightPitch-val is deprecated.  Use pau2motors-msg:m_eyeGazeRightPitch instead.")
  (m_eyeGazeRightPitch m))

(cl:ensure-generic-function 'm_eyeGazeRightYaw-val :lambda-list '(m))
(cl:defmethod m_eyeGazeRightYaw-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_eyeGazeRightYaw-val is deprecated.  Use pau2motors-msg:m_eyeGazeRightYaw instead.")
  (m_eyeGazeRightYaw m))

(cl:ensure-generic-function 'm_coeffs-val :lambda-list '(m))
(cl:defmethod m_coeffs-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_coeffs-val is deprecated.  Use pau2motors-msg:m_coeffs instead.")
  (m_coeffs m))

(cl:ensure-generic-function 'm_shapekeys-val :lambda-list '(m))
(cl:defmethod m_shapekeys-val ((m <pau>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pau2motors-msg:m_shapekeys-val is deprecated.  Use pau2motors-msg:m_shapekeys instead.")
  (m_shapekeys m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pau>) ostream)
  "Serializes a message object of type '<pau>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm_headRotation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm_headTranslation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'm_neckRotation) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm_eyeGazeLeftPitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm_eyeGazeLeftYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm_eyeGazeRightPitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm_eyeGazeRightYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'm_coeffs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'm_coeffs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'm_shapekeys))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'm_shapekeys))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pau>) istream)
  "Deserializes a message object of type '<pau>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm_headRotation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm_headTranslation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'm_neckRotation) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm_eyeGazeLeftPitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm_eyeGazeLeftYaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm_eyeGazeRightPitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm_eyeGazeRightYaw) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'm_coeffs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'm_coeffs)))
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
  (cl:setf (cl:slot-value msg 'm_shapekeys) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'm_shapekeys)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pau>)))
  "Returns string type for a message object of type '<pau>"
  "pau2motors/pau")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pau)))
  "Returns string type for a message object of type 'pau"
  "pau2motors/pau")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pau>)))
  "Returns md5sum for a message object of type '<pau>"
  "49c3e3c79051e845f49e85a5ebffa67a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pau)))
  "Returns md5sum for a message object of type 'pau"
  "49c3e3c79051e845f49e85a5ebffa67a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pau>)))
  "Returns full string definition for message of type '<pau>"
  (cl:format cl:nil "geometry_msgs/Quaternion m_headRotation~%geometry_msgs/Vector3    m_headTranslation~%geometry_msgs/Quaternion m_neckRotation~%~%float32 m_eyeGazeLeftPitch~%float32 m_eyeGazeLeftYaw~%float32 m_eyeGazeRightPitch~%float32 m_eyeGazeRightYaw~%~%#An array of blendshape coefficients.~%#They describe an expression of a virtual face.~%float32[] m_coeffs~%# Then setting shapekey values, names must be passed~%string[] m_shapekeys~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pau)))
  "Returns full string definition for message of type 'pau"
  (cl:format cl:nil "geometry_msgs/Quaternion m_headRotation~%geometry_msgs/Vector3    m_headTranslation~%geometry_msgs/Quaternion m_neckRotation~%~%float32 m_eyeGazeLeftPitch~%float32 m_eyeGazeLeftYaw~%float32 m_eyeGazeRightPitch~%float32 m_eyeGazeRightYaw~%~%#An array of blendshape coefficients.~%#They describe an expression of a virtual face.~%float32[] m_coeffs~%# Then setting shapekey values, names must be passed~%string[] m_shapekeys~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pau>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm_headRotation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm_headTranslation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'm_neckRotation))
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'm_coeffs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'm_shapekeys) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pau>))
  "Converts a ROS message object to a list"
  (cl:list 'pau
    (cl:cons ':m_headRotation (m_headRotation msg))
    (cl:cons ':m_headTranslation (m_headTranslation msg))
    (cl:cons ':m_neckRotation (m_neckRotation msg))
    (cl:cons ':m_eyeGazeLeftPitch (m_eyeGazeLeftPitch msg))
    (cl:cons ':m_eyeGazeLeftYaw (m_eyeGazeLeftYaw msg))
    (cl:cons ':m_eyeGazeRightPitch (m_eyeGazeRightPitch msg))
    (cl:cons ':m_eyeGazeRightYaw (m_eyeGazeRightYaw msg))
    (cl:cons ':m_coeffs (m_coeffs msg))
    (cl:cons ':m_shapekeys (m_shapekeys msg))
))
