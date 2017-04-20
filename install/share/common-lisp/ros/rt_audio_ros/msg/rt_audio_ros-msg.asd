
(cl:in-package :asdf)

(defsystem "rt_audio_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AudioStream" :depends-on ("_package_AudioStream"))
    (:file "_package_AudioStream" :depends-on ("_package"))
  ))