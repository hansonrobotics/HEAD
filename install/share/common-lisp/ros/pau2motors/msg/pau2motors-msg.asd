
(cl:in-package :asdf)

(defsystem "pau2motors-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "pau" :depends-on ("_package_pau"))
    (:file "_package_pau" :depends-on ("_package"))
  ))