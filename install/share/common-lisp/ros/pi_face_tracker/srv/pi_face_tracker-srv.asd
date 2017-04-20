
(cl:in-package :asdf)

(defsystem "pi_face_tracker-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "KeyCommand" :depends-on ("_package_KeyCommand"))
    (:file "_package_KeyCommand" :depends-on ("_package"))
    (:file "SetROI" :depends-on ("_package_SetROI"))
    (:file "_package_SetROI" :depends-on ("_package"))
  ))