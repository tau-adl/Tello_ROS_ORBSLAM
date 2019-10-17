; Auto-generated. Do not edit!


(cl:in-package svo_msgs-msg)


;//! \htmlinclude Info.msg.html

(cl:defclass <Info> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (processing_time
    :reader processing_time
    :initarg :processing_time
    :type cl:float
    :initform 0.0)
   (keyframes
    :reader keyframes
    :initarg :keyframes
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (num_matches
    :reader num_matches
    :initarg :num_matches
    :type cl:integer
    :initform 0)
   (tracking_quality
    :reader tracking_quality
    :initarg :tracking_quality
    :type cl:integer
    :initform 0)
   (stage
    :reader stage
    :initarg :stage
    :type cl:integer
    :initform 0))
)

(cl:defclass Info (<Info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name svo_msgs-msg:<Info> is deprecated: use svo_msgs-msg:Info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:header-val is deprecated.  Use svo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'processing_time-val :lambda-list '(m))
(cl:defmethod processing_time-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:processing_time-val is deprecated.  Use svo_msgs-msg:processing_time instead.")
  (processing_time m))

(cl:ensure-generic-function 'keyframes-val :lambda-list '(m))
(cl:defmethod keyframes-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:keyframes-val is deprecated.  Use svo_msgs-msg:keyframes instead.")
  (keyframes m))

(cl:ensure-generic-function 'num_matches-val :lambda-list '(m))
(cl:defmethod num_matches-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:num_matches-val is deprecated.  Use svo_msgs-msg:num_matches instead.")
  (num_matches m))

(cl:ensure-generic-function 'tracking_quality-val :lambda-list '(m))
(cl:defmethod tracking_quality-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:tracking_quality-val is deprecated.  Use svo_msgs-msg:tracking_quality instead.")
  (tracking_quality m))

(cl:ensure-generic-function 'stage-val :lambda-list '(m))
(cl:defmethod stage-val ((m <Info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:stage-val is deprecated.  Use svo_msgs-msg:stage instead.")
  (stage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Info>) ostream)
  "Serializes a message object of type '<Info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'processing_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keyframes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'keyframes))
  (cl:let* ((signed (cl:slot-value msg 'num_matches)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tracking_quality)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Info>) istream)
  "Deserializes a message object of type '<Info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'processing_time) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keyframes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keyframes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_matches) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tracking_quality) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stage) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Info>)))
  "Returns string type for a message object of type '<Info>"
  "svo_msgs/Info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Info)))
  "Returns string type for a message object of type 'Info"
  "svo_msgs/Info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Info>)))
  "Returns md5sum for a message object of type '<Info>"
  "175acf2e539a9219addbcbeafca8552f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Info)))
  "Returns md5sum for a message object of type 'Info"
  "175acf2e539a9219addbcbeafca8552f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Info>)))
  "Returns full string definition for message of type '<Info>"
  (cl:format cl:nil "Header      header~%float32     processing_time~%int32[]     keyframes~%int32       num_matches~%int32       tracking_quality~%int32       stage~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Info)))
  "Returns full string definition for message of type 'Info"
  (cl:format cl:nil "Header      header~%float32     processing_time~%int32[]     keyframes~%int32       num_matches~%int32       tracking_quality~%int32       stage~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keyframes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Info>))
  "Converts a ROS message object to a list"
  (cl:list 'Info
    (cl:cons ':header (header msg))
    (cl:cons ':processing_time (processing_time msg))
    (cl:cons ':keyframes (keyframes msg))
    (cl:cons ':num_matches (num_matches msg))
    (cl:cons ':tracking_quality (tracking_quality msg))
    (cl:cons ':stage (stage msg))
))
