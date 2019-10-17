
(cl:in-package :asdf)

(defsystem "flock_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FlightData" :depends-on ("_package_FlightData"))
    (:file "_package_FlightData" :depends-on ("_package"))
    (:file "Flip" :depends-on ("_package_Flip"))
    (:file "_package_Flip" :depends-on ("_package"))
  ))