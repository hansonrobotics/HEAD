
(cl:in-package :asdf)

(defsystem "tts-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TTSLength" :depends-on ("_package_TTSLength"))
    (:file "_package_TTSLength" :depends-on ("_package"))
  ))