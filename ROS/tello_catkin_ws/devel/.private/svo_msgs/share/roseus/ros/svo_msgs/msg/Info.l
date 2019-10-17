;; Auto-generated. Do not edit!


(when (boundp 'svo_msgs::Info)
  (if (not (find-package "SVO_MSGS"))
    (make-package "SVO_MSGS"))
  (shadow 'Info (find-package "SVO_MSGS")))
(unless (find-package "SVO_MSGS::INFO")
  (make-package "SVO_MSGS::INFO"))

(in-package "ROS")
;;//! \htmlinclude Info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass svo_msgs::Info
  :super ros::object
  :slots (_header _processing_time _keyframes _num_matches _tracking_quality _stage ))

(defmethod svo_msgs::Info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:processing_time __processing_time) 0.0)
    ((:keyframes __keyframes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:num_matches __num_matches) 0)
    ((:tracking_quality __tracking_quality) 0)
    ((:stage __stage) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _processing_time (float __processing_time))
   (setq _keyframes __keyframes)
   (setq _num_matches (round __num_matches))
   (setq _tracking_quality (round __tracking_quality))
   (setq _stage (round __stage))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:processing_time
   (&optional __processing_time)
   (if __processing_time (setq _processing_time __processing_time)) _processing_time)
  (:keyframes
   (&optional __keyframes)
   (if __keyframes (setq _keyframes __keyframes)) _keyframes)
  (:num_matches
   (&optional __num_matches)
   (if __num_matches (setq _num_matches __num_matches)) _num_matches)
  (:tracking_quality
   (&optional __tracking_quality)
   (if __tracking_quality (setq _tracking_quality __tracking_quality)) _tracking_quality)
  (:stage
   (&optional __stage)
   (if __stage (setq _stage __stage)) _stage)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _processing_time
    4
    ;; int32[] _keyframes
    (* 4    (length _keyframes)) 4
    ;; int32 _num_matches
    4
    ;; int32 _tracking_quality
    4
    ;; int32 _stage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _processing_time
       (sys::poke _processing_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32[] _keyframes
     (write-long (length _keyframes) s)
     (dotimes (i (length _keyframes))
       (write-long (elt _keyframes i) s)
       )
     ;; int32 _num_matches
       (write-long _num_matches s)
     ;; int32 _tracking_quality
       (write-long _tracking_quality s)
     ;; int32 _stage
       (write-long _stage s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _processing_time
     (setq _processing_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32[] _keyframes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keyframes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _keyframes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _num_matches
     (setq _num_matches (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _tracking_quality
     (setq _tracking_quality (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _stage
     (setq _stage (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get svo_msgs::Info :md5sum-) "175acf2e539a9219addbcbeafca8552f")
(setf (get svo_msgs::Info :datatype-) "svo_msgs/Info")
(setf (get svo_msgs::Info :definition-)
      "Header      header
float32     processing_time
int32[]     keyframes
int32       num_matches
int32       tracking_quality
int32       stage
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :svo_msgs/Info "175acf2e539a9219addbcbeafca8552f")


