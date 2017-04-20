; Auto-generated. Do not edit!


(cl:in-package facial_puppetry-msg)


;//! \htmlinclude land_marks.msg.html

(cl:defclass <land_marks> (roslisp-msg-protocol:ros-message)
  ((dlib_val
    :reader dlib_val
    :initarg :dlib_val
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (max_ref
    :reader max_ref
    :initarg :max_ref
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (distX
    :reader distX
    :initarg :distX
    :type cl:float
    :initform 0.0)
   (distY
    :reader distY
    :initarg :distY
    :type cl:float
    :initform 0.0)
   (distW
    :reader distW
    :initarg :distW
    :type cl:float
    :initform 0.0)
   (distH
    :reader distH
    :initarg :distH
    :type cl:float
    :initform 0.0))
)

(cl:defclass land_marks (<land_marks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <land_marks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'land_marks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name facial_puppetry-msg:<land_marks> is deprecated: use facial_puppetry-msg:land_marks instead.")))

(cl:ensure-generic-function 'dlib_val-val :lambda-list '(m))
(cl:defmethod dlib_val-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:dlib_val-val is deprecated.  Use facial_puppetry-msg:dlib_val instead.")
  (dlib_val m))

(cl:ensure-generic-function 'max_ref-val :lambda-list '(m))
(cl:defmethod max_ref-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:max_ref-val is deprecated.  Use facial_puppetry-msg:max_ref instead.")
  (max_ref m))

(cl:ensure-generic-function 'distX-val :lambda-list '(m))
(cl:defmethod distX-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:distX-val is deprecated.  Use facial_puppetry-msg:distX instead.")
  (distX m))

(cl:ensure-generic-function 'distY-val :lambda-list '(m))
(cl:defmethod distY-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:distY-val is deprecated.  Use facial_puppetry-msg:distY instead.")
  (distY m))

(cl:ensure-generic-function 'distW-val :lambda-list '(m))
(cl:defmethod distW-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:distW-val is deprecated.  Use facial_puppetry-msg:distW instead.")
  (distW m))

(cl:ensure-generic-function 'distH-val :lambda-list '(m))
(cl:defmethod distH-val ((m <land_marks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader facial_puppetry-msg:distH-val is deprecated.  Use facial_puppetry-msg:distH instead.")
  (distH m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <land_marks>) ostream)
  "Serializes a message object of type '<land_marks>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dlib_val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'dlib_val))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'max_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'max_ref))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distW))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <land_marks>) istream)
  "Deserializes a message object of type '<land_marks>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dlib_val) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dlib_val)))
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
  (cl:setf (cl:slot-value msg 'max_ref) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'max_ref)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distW) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distH) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<land_marks>)))
  "Returns string type for a message object of type '<land_marks>"
  "facial_puppetry/land_marks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'land_marks)))
  "Returns string type for a message object of type 'land_marks"
  "facial_puppetry/land_marks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<land_marks>)))
  "Returns md5sum for a message object of type '<land_marks>"
  "f6d1fcaa4d346ffd01a06e016371d2ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'land_marks)))
  "Returns md5sum for a message object of type 'land_marks"
  "f6d1fcaa4d346ffd01a06e016371d2ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<land_marks>)))
  "Returns full string definition for message of type '<land_marks>"
  (cl:format cl:nil "float32[] dlib_val~%float32[] max_ref~%float32 distX~%float32 distY~%float32 distW~%float32 distH~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'land_marks)))
  "Returns full string definition for message of type 'land_marks"
  (cl:format cl:nil "float32[] dlib_val~%float32[] max_ref~%float32 distX~%float32 distY~%float32 distW~%float32 distH~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <land_marks>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dlib_val) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'max_ref) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <land_marks>))
  "Converts a ROS message object to a list"
  (cl:list 'land_marks
    (cl:cons ':dlib_val (dlib_val msg))
    (cl:cons ':max_ref (max_ref msg))
    (cl:cons ':distX (distX msg))
    (cl:cons ':distY (distY msg))
    (cl:cons ':distW (distW msg))
    (cl:cons ':distH (distH msg))
))
