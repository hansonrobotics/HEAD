
(cl:in-package :asdf)

(defsystem "blender_api_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetParam" :depends-on ("_package_GetParam"))
    (:file "_package_GetParam" :depends-on ("_package"))
    (:file "GetAnimationLength" :depends-on ("_package_GetAnimationLength"))
    (:file "_package_GetAnimationLength" :depends-on ("_package"))
    (:file "SetParam" :depends-on ("_package_SetParam"))
    (:file "_package_SetParam" :depends-on ("_package"))
  ))