
(cl:in-package :asdf)

(defsystem "orb_slam2_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SaveMap" :depends-on ("_package_SaveMap"))
    (:file "_package_SaveMap" :depends-on ("_package"))
  ))