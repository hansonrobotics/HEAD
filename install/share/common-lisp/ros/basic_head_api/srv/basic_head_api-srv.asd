
(cl:in-package :asdf)

(defsystem "basic_head_api-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ValidFaceExprs" :depends-on ("_package_ValidFaceExprs"))
    (:file "_package_ValidFaceExprs" :depends-on ("_package"))
    (:file "AnimationLength" :depends-on ("_package_AnimationLength"))
    (:file "_package_AnimationLength" :depends-on ("_package"))
  ))