;; Auto-generated. Do not edit!


(when (boundp 'svo_msgs::Feature)
  (if (not (find-package "SVO_MSGS"))
    (make-package "SVO_MSGS"))
  (shadow 'Feature (find-package "SVO_MSGS")))
(unless (find-package "SVO_MSGS::FEATURE")
  (make-package "SVO_MSGS::FEATURE"))

(in-package "ROS")
;;//! \htmlinclude Feature.msg.html


(defclass svo_msgs::Feature
  :super ros::object
  :slots (_u _v _x _y _z ))

(defmethod svo_msgs::Feature
  (:init
   (&key
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _u
    4
    ;; float32 _v
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _u
       (sys::poke _u (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _u
     (setq _u (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get svo_msgs::Feature :md5sum-) "4f7761c191bddfddcd2f99bea993a700")
(setf (get svo_msgs::Feature :datatype-) "svo_msgs/Feature")
(setf (get svo_msgs::Feature :definition-)
      "float32 u
float32 v
float32 x
float32 y
float32 z
")



(provide :svo_msgs/Feature "4f7761c191bddfddcd2f99bea993a700")


