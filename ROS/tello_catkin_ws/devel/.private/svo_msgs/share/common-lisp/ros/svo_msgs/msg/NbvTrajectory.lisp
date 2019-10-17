; Auto-generated. Do not edit!


(cl:in-package svo_msgs-msg)


;//! \htmlinclude NbvTrajectory.msg.html

(cl:defclass <NbvTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass NbvTrajectory (<NbvTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NbvTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NbvTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name svo_msgs-msg:<NbvTrajectory> is deprecated: use svo_msgs-msg:NbvTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NbvTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:header-val is deprecated.  Use svo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <NbvTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:trajectory-val is deprecated.  Use svo_msgs-msg:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NbvTrajectory>) ostream)
  "Serializes a message object of type '<NbvTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NbvTrajectory>) istream)
  "Deserializes a message object of type '<NbvTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NbvTrajectory>)))
  "Returns string type for a message object of type '<NbvTrajectory>"
  "svo_msgs/NbvTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NbvTrajectory)))
  "Returns string type for a message object of type 'NbvTrajectory"
  "svo_msgs/NbvTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NbvTrajectory>)))
  "Returns md5sum for a message object of type '<NbvTrajectory>"
  "eb6b516341d535312e1b3849a61711c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NbvTrajectory)))
  "Returns md5sum for a message object of type 'NbvTrajectory"
  "eb6b516341d535312e1b3849a61711c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NbvTrajectory>)))
  "Returns full string definition for message of type '<NbvTrajectory>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] trajectory~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NbvTrajectory)))
  "Returns full string definition for message of type 'NbvTrajectory"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose[] trajectory~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NbvTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NbvTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'NbvTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':trajectory (trajectory msg))
))
