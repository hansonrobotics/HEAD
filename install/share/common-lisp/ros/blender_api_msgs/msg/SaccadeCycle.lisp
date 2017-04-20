; Auto-generated. Do not edit!


(cl:in-package blender_api_msgs-msg)


;//! \htmlinclude SaccadeCycle.msg.html

(cl:defclass <SaccadeCycle> (roslisp-msg-protocol:ros-message)
  ((mean
    :reader mean
    :initarg :mean
    :type cl:float
    :initform 0.0)
   (variation
    :reader variation
    :initarg :variation
    :type cl:float
    :initform 0.0)
   (paint_scale
    :reader paint_scale
    :initarg :paint_scale
    :type cl:float
    :initform 0.0)
   (eye_size
    :reader eye_size
    :initarg :eye_size
    :type cl:float
    :initform 0.0)
   (eye_distance
    :reader eye_distance
    :initarg :eye_distance
    :type cl:float
    :initform 0.0)
   (mouth_width
    :reader mouth_width
    :initarg :mouth_width
    :type cl:float
    :initform 0.0)
   (mouth_height
    :reader mouth_height
    :initarg :mouth_height
    :type cl:float
    :initform 0.0)
   (weight_eyes
    :reader weight_eyes
    :initarg :weight_eyes
    :type cl:float
    :initform 0.0)
   (weight_mouth
    :reader weight_mouth
    :initarg :weight_mouth
    :type cl:float
    :initform 0.0))
)

(cl:defclass SaccadeCycle (<SaccadeCycle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaccadeCycle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaccadeCycle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name blender_api_msgs-msg:<SaccadeCycle> is deprecated: use blender_api_msgs-msg:SaccadeCycle instead.")))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:mean-val is deprecated.  Use blender_api_msgs-msg:mean instead.")
  (mean m))

(cl:ensure-generic-function 'variation-val :lambda-list '(m))
(cl:defmethod variation-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:variation-val is deprecated.  Use blender_api_msgs-msg:variation instead.")
  (variation m))

(cl:ensure-generic-function 'paint_scale-val :lambda-list '(m))
(cl:defmethod paint_scale-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:paint_scale-val is deprecated.  Use blender_api_msgs-msg:paint_scale instead.")
  (paint_scale m))

(cl:ensure-generic-function 'eye_size-val :lambda-list '(m))
(cl:defmethod eye_size-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:eye_size-val is deprecated.  Use blender_api_msgs-msg:eye_size instead.")
  (eye_size m))

(cl:ensure-generic-function 'eye_distance-val :lambda-list '(m))
(cl:defmethod eye_distance-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:eye_distance-val is deprecated.  Use blender_api_msgs-msg:eye_distance instead.")
  (eye_distance m))

(cl:ensure-generic-function 'mouth_width-val :lambda-list '(m))
(cl:defmethod mouth_width-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:mouth_width-val is deprecated.  Use blender_api_msgs-msg:mouth_width instead.")
  (mouth_width m))

(cl:ensure-generic-function 'mouth_height-val :lambda-list '(m))
(cl:defmethod mouth_height-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:mouth_height-val is deprecated.  Use blender_api_msgs-msg:mouth_height instead.")
  (mouth_height m))

(cl:ensure-generic-function 'weight_eyes-val :lambda-list '(m))
(cl:defmethod weight_eyes-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:weight_eyes-val is deprecated.  Use blender_api_msgs-msg:weight_eyes instead.")
  (weight_eyes m))

(cl:ensure-generic-function 'weight_mouth-val :lambda-list '(m))
(cl:defmethod weight_mouth-val ((m <SaccadeCycle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader blender_api_msgs-msg:weight_mouth-val is deprecated.  Use blender_api_msgs-msg:weight_mouth instead.")
  (weight_mouth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaccadeCycle>) ostream)
  "Serializes a message object of type '<SaccadeCycle>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'variation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'paint_scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'eye_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'eye_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mouth_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mouth_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight_eyes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight_mouth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaccadeCycle>) istream)
  "Deserializes a message object of type '<SaccadeCycle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'variation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'paint_scale) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'eye_size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'eye_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mouth_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mouth_height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight_eyes) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight_mouth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaccadeCycle>)))
  "Returns string type for a message object of type '<SaccadeCycle>"
  "blender_api_msgs/SaccadeCycle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaccadeCycle)))
  "Returns string type for a message object of type 'SaccadeCycle"
  "blender_api_msgs/SaccadeCycle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaccadeCycle>)))
  "Returns md5sum for a message object of type '<SaccadeCycle>"
  "2ddd4aa1af1ce0e41299d9e0d97ba48f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaccadeCycle)))
  "Returns md5sum for a message object of type 'SaccadeCycle"
  "2ddd4aa1af1ce0e41299d9e0d97ba48f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaccadeCycle>)))
  "Returns full string definition for message of type '<SaccadeCycle>"
  (cl:format cl:nil "float32 mean~%float32 variation~%float32 paint_scale~%float32 eye_size~%float32 eye_distance~%float32 mouth_width~%float32 mouth_height~%float32 weight_eyes~%float32 weight_mouth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaccadeCycle)))
  "Returns full string definition for message of type 'SaccadeCycle"
  (cl:format cl:nil "float32 mean~%float32 variation~%float32 paint_scale~%float32 eye_size~%float32 eye_distance~%float32 mouth_width~%float32 mouth_height~%float32 weight_eyes~%float32 weight_mouth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaccadeCycle>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaccadeCycle>))
  "Converts a ROS message object to a list"
  (cl:list 'SaccadeCycle
    (cl:cons ':mean (mean msg))
    (cl:cons ':variation (variation msg))
    (cl:cons ':paint_scale (paint_scale msg))
    (cl:cons ':eye_size (eye_size msg))
    (cl:cons ':eye_distance (eye_distance msg))
    (cl:cons ':mouth_width (mouth_width msg))
    (cl:cons ':mouth_height (mouth_height msg))
    (cl:cons ':weight_eyes (weight_eyes msg))
    (cl:cons ':weight_mouth (weight_mouth msg))
))
