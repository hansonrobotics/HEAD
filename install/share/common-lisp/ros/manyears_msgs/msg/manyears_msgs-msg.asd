
(cl:in-package :asdf)

(defsystem "manyears_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SourceInfo" :depends-on ("_package_SourceInfo"))
    (:file "_package_SourceInfo" :depends-on ("_package"))
    (:file "SourceInfoWithCovariance" :depends-on ("_package_SourceInfoWithCovariance"))
    (:file "_package_SourceInfoWithCovariance" :depends-on ("_package"))
    (:file "ManyEarsTrackedAudioSource" :depends-on ("_package_ManyEarsTrackedAudioSource"))
    (:file "_package_ManyEarsTrackedAudioSource" :depends-on ("_package"))
  ))