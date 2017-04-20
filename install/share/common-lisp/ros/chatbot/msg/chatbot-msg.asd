
(cl:in-package :asdf)

(defsystem "chatbot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChatMessage" :depends-on ("_package_ChatMessage"))
    (:file "_package_ChatMessage" :depends-on ("_package"))
  ))