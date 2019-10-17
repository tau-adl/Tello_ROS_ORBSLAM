;; Auto-generated. Do not edit!


(when (boundp 'orb_slam2_ros::SaveMap)
  (if (not (find-package "ORB_SLAM2_ROS"))
    (make-package "ORB_SLAM2_ROS"))
  (shadow 'SaveMap (find-package "ORB_SLAM2_ROS")))
(unless (find-package "ORB_SLAM2_ROS::SAVEMAP")
  (make-package "ORB_SLAM2_ROS::SAVEMAP"))
(unless (find-package "ORB_SLAM2_ROS::SAVEMAPREQUEST")
  (make-package "ORB_SLAM2_ROS::SAVEMAPREQUEST"))
(unless (find-package "ORB_SLAM2_ROS::SAVEMAPRESPONSE")
  (make-package "ORB_SLAM2_ROS::SAVEMAPRESPONSE"))

(in-package "ROS")





(defclass orb_slam2_ros::SaveMapRequest
  :super ros::object
  :slots (_name ))

(defmethod orb_slam2_ros::SaveMapRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass orb_slam2_ros::SaveMapResponse
  :super ros::object
  :slots (_success ))

(defmethod orb_slam2_ros::SaveMapResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass orb_slam2_ros::SaveMap
  :super ros::object
  :slots ())

(setf (get orb_slam2_ros::SaveMap :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get orb_slam2_ros::SaveMap :datatype-) "orb_slam2_ros/SaveMap")
(setf (get orb_slam2_ros::SaveMap :request) orb_slam2_ros::SaveMapRequest)
(setf (get orb_slam2_ros::SaveMap :response) orb_slam2_ros::SaveMapResponse)

(defmethod orb_slam2_ros::SaveMapRequest
  (:response () (instance orb_slam2_ros::SaveMapResponse :init)))

(setf (get orb_slam2_ros::SaveMapRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get orb_slam2_ros::SaveMapRequest :datatype-) "orb_slam2_ros/SaveMapRequest")
(setf (get orb_slam2_ros::SaveMapRequest :definition-)
      "string name
---
bool success
")

(setf (get orb_slam2_ros::SaveMapResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get orb_slam2_ros::SaveMapResponse :datatype-) "orb_slam2_ros/SaveMapResponse")
(setf (get orb_slam2_ros::SaveMapResponse :definition-)
      "string name
---
bool success
")



(provide :orb_slam2_ros/SaveMap "d08a3b641c2f8680fbdfb1ea2e17a3e1")


