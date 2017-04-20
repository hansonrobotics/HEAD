
(cl:in-package :asdf)

(defsystem "basic_head_api-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PlayAnimation" :depends-on ("_package_PlayAnimation"))
    (:file "_package_PlayAnimation" :depends-on ("_package"))
    (:file "MakeFaceExpr" :depends-on ("_package_MakeFaceExpr"))
    (:file "_package_MakeFaceExpr" :depends-on ("_package"))
    (:file "PointHead" :depends-on ("_package_PointHead"))
    (:file "_package_PointHead" :depends-on ("_package"))
  ))