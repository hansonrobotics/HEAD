
(cl:in-package :asdf)

(defsystem "pi_face_tracker-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Faces" :depends-on ("_package_Faces"))
    (:file "_package_Faces" :depends-on ("_package"))
    (:file "Face" :depends-on ("_package_Face"))
    (:file "_package_Face" :depends-on ("_package"))
    (:file "FaceEvent" :depends-on ("_package_FaceEvent"))
    (:file "_package_FaceEvent" :depends-on ("_package"))
  ))