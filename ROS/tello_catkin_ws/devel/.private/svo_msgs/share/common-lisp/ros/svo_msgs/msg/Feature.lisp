; Auto-generated. Do not edit!


(cl:in-package svo_msgs-msg)


;//! \htmlinclude Feature.msg.html

(cl:defclass <Feature> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Feature (<Feature>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Feature>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Feature)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name svo_msgs-msg:<Feature> is deprecated: use svo_msgs-msg:Feature instead.")))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <Feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:u-val is deprecated.  Use svo_msgs-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <Feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:v-val is deprecated.  Use svo_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:x-val is deprecated.  Use svo_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:y-val is deprecated.  Use svo_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Feature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader svo_msgs-msg:z-val is deprecated.  Use svo_msgs-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Feature>) ostream)
  "Serializes a message object of type '<Feature>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Feature>) istream)
  "Deserializes a message object of type '<Feature>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Feature>)))
  "Returns string type for a message object of type '<Feature>"
  "svo_msgs/Feature")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Feature)))
  "Returns string type for a message object of type 'Feature"
  "svo_msgs/Feature")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Feature>)))
  "Returns md5sum for a message object of type '<Feature>"
  "4f7761c191bddfddcd2f99bea993a700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Feature)))
  "Returns md5sum for a message object of type 'Feature"
  "4f7761c191bddfddcd2f99bea993a700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Feature>)))
  "Returns full string definition for message of type '<Feature>"
  (cl:format cl:nil "float32 u~%float32 v~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Feature)))
  "Returns full string definition for message of type 'Feature"
  (cl:format cl:nil "float32 u~%float32 v~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Feature>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Feature>))
  "Converts a ROS message object to a list"
  (cl:list 'Feature
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
