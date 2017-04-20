
(cl:in-package :asdf)

(defsystem "facial_puppetry-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "land_marks" :depends-on ("_package_land_marks"))
    (:file "_package_land_marks" :depends-on ("_package"))
  ))