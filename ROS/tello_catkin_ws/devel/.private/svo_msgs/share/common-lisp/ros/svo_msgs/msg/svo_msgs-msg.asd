
(cl:in-package :asdf)

(defsystem "svo_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DenseInput" :depends-on ("_package_DenseInput"))
    (:file "_package_DenseInput" :depends-on ("_package"))
    (:file "Feature" :depends-on ("_package_Feature"))
    (:file "_package_Feature" :depends-on ("_package"))
    (:file "Info" :depends-on ("_package_Info"))
    (:file "_package_Info" :depends-on ("_package"))
    (:file "NbvTrajectory" :depends-on ("_package_NbvTrajectory"))
    (:file "_package_NbvTrajectory" :depends-on ("_package"))
  ))