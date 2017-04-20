
(cl:in-package :asdf)

(defsystem "webui-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NodeConfiguration" :depends-on ("_package_NodeConfiguration"))
    (:file "_package_NodeConfiguration" :depends-on ("_package"))
    (:file "MotorStates" :depends-on ("_package_MotorStates"))
    (:file "_package_MotorStates" :depends-on ("_package"))
    (:file "Json" :depends-on ("_package_Json"))
    (:file "_package_Json" :depends-on ("_package"))
    (:file "ConfigurableNodes" :depends-on ("_package_ConfigurableNodes"))
    (:file "_package_ConfigurableNodes" :depends-on ("_package"))
    (:file "NodeDescription" :depends-on ("_package_NodeDescription"))
    (:file "_package_NodeDescription" :depends-on ("_package"))
  ))