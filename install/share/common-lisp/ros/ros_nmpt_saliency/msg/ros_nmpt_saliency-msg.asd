
(cl:in-package :asdf)

(defsystem "ros_nmpt_saliency-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "targets" :depends-on ("_package_targets"))
    (:file "_package_targets" :depends-on ("_package"))
  ))