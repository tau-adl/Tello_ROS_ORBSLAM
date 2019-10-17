; Auto-generated. Do not edit!


(cl:in-package svo_msgs-msg)


;//! \htmlinclude DenseInput.msg.html

(cl:defclass <DenseInput> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (min_depth
    :reader min_depth
    :initarg :min_depth
    :type cl:float
    :initform 0.0)
   (max_depth
    :reader max_depth
    :initarg :max_depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass DenseInput (<DenseInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DenseInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DenseInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name svo_msgs-msg:<DenseInput> is deprecated: use svo_msgs-msg:DenseInput instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:header-val is deprecated.  Use svo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:frame_id-val is deprecated.  Use svo_msgs-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:pose-val is deprecated.  Use svo_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:image-val is deprecated.  Use svo_msgs-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'min_depth-val :lambda-list '(m))
(cl:defmethod min_depth-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:min_depth-val is deprecated.  Use svo_msgs-msg:min_depth instead.")
  (min_depth m))

(cl:ensure-generic-function 'max_depth-val :lambda-list '(m))
(cl:defmethod max_depth-val ((m <DenseInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:max_depth-val is deprecated.  Use svo_msgs-msg:max_depth instead.")
  (max_depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DenseInput>) ostream)
  "Serializes a message object of type '<DenseInput>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DenseInput>) istream)
  "Deserializes a message object of type '<DenseInput>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_depth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_depth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DenseInput>)))
  "Returns string type for a message object of type '<DenseInput>"
  "svo_msgs/DenseInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DenseInput)))
  "Returns string type for a message object of type 'DenseInput"
  "svo_msgs/DenseInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DenseInput>)))
  "Returns md5sum for a message object of type '<DenseInput>"
  "cea677f47dcf08581cc9f5efece2f7e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DenseInput)))
  "Returns md5sum for a message object of type 'DenseInput"
  "cea677f47dcf08581cc9f5efece2f7e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DenseInput>)))
  "Returns full string definition for message of type '<DenseInput>"
  (cl:format cl:nil "Header header~%uint32 frame_id~%geometry_msgs/Pose pose~%sensor_msgs/Image image~%float32 min_depth~%float32 max_depth~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DenseInput)))
  "Returns full string definition for message of type 'DenseInput"
  (cl:format cl:nil "Header header~%uint32 frame_id~%geometry_msgs/Pose pose~%sensor_msgs/Image image~%float32 min_depth~%float32 max_depth~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DenseInput>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DenseInput>))
  "Converts a ROS message object to a list"
  (cl:list 'DenseInput
    (cl:cons ':header (header msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':image (image msg))
    (cl:cons ':min_depth (min_depth msg))
    (cl:cons ':max_depth (max_depth msg))
))
