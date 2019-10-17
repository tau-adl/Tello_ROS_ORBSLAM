;; Auto-generated. Do not edit!


(when (boundp 'flock_msgs::Flip)
  (if (not (find-package "FLOCK_MSGS"))
    (make-package "FLOCK_MSGS"))
  (shadow 'Flip (find-package "FLOCK_MSGS")))
(unless (find-package "FLOCK_MSGS::FLIP")
  (make-package "FLOCK_MSGS::FLIP"))

(in-package "ROS")
;;//! \htmlinclude Flip.msg.html


(intern "*FLIP_FORWARD*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_FORWARD* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_FORWARD* 0)
(intern "*FLIP_LEFT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_LEFT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_LEFT* 1)
(intern "*FLIP_BACK*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_BACK* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_BACK* 2)
(intern "*FLIP_RIGHT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_RIGHT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_RIGHT* 3)
(intern "*FLIP_FORWARDLEFT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_FORWARDLEFT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_FORWARDLEFT* 4)
(intern "*FLIP_BACKLEFT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_BACKLEFT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_BACKLEFT* 5)
(intern "*FLIP_BACKRIGHT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_BACKRIGHT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_BACKRIGHT* 6)
(intern "*FLIP_FORWARDRIGHT*" (find-package "FLOCK_MSGS::FLIP"))
(shadow '*FLIP_FORWARDRIGHT* (find-package "FLOCK_MSGS::FLIP"))
(defconstant flock_msgs::Flip::*FLIP_FORWARDRIGHT* 7)
(defclass flock_msgs::Flip
  :super ros::object
  :slots (_flip_command ))

(defmethod flock_msgs::Flip
  (:init
   (&key
    ((:flip_command __flip_command) 0)
    )
   (send-super :init)
   (setq _flip_command (round __flip_command))
   self)
  (:flip_command
   (&optional __flip_command)
   (if __flip_command (setq _flip_command __flip_command)) _flip_command)
  (:serialization-length
   ()
   (+
    ;; uint8 _flip_command
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _flip_command
       (write-byte _flip_command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _flip_command
     (setq _flip_command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get flock_msgs::Flip :md5sum-) "900021b0eeb6e5c98ddc735ee4fde90e")
(setf (get flock_msgs::Flip :datatype-) "flock_msgs/Flip")
(setf (get flock_msgs::Flip :definition-)
      "# Names and values match TelloPy
uint8 flip_forward=0
uint8 flip_left=1
uint8 flip_back=2
uint8 flip_right=3
uint8 flip_forwardleft=4
uint8 flip_backleft=5
uint8 flip_backright=6
uint8 flip_forwardright=7
uint8 flip_command

")



(provide :flock_msgs/Flip "900021b0eeb6e5c98ddc735ee4fde90e")


