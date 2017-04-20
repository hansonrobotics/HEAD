
(cl:in-package :asdf)

(defsystem "room_luminance-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Luminance" :depends-on ("_package_Luminance"))
    (:file "_package_Luminance" :depends-on ("_package"))
  ))