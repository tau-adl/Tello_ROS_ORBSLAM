; Auto-generated. Do not edit!


(cl:in-package flock_msgs-msg)


;//! \htmlinclude FlightData.msg.html

(cl:defclass <FlightData> (roslisp-msg-protocol:ros-message)
  ((battery_percent
    :reader battery_percent
    :initarg :battery_percent
    :type cl:integer
    :initform 0)
   (estimated_flight_time_remaining
    :reader estimated_flight_time_remaining
    :initarg :estimated_flight_time_remaining
    :type cl:float
    :initform 0.0)
   (flight_mode
    :reader flight_mode
    :initarg :flight_mode
    :type cl:fixnum
    :initform 0)
   (flight_time
    :reader flight_time
    :initarg :flight_time
    :type cl:float
    :initform 0.0)
   (east_speed
    :reader east_speed
    :initarg :east_speed
    :type cl:float
    :initform 0.0)
   (north_speed
    :reader north_speed
    :initarg :north_speed
    :type cl:float
    :initform 0.0)
   (ground_speed
    :reader ground_speed
    :initarg :ground_speed
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (equipment
    :reader equipment
    :initarg :equipment
    :type cl:integer
    :initform 0)
   (high_temperature
    :reader high_temperature
    :initarg :high_temperature
    :type cl:boolean
    :initform cl:nil)
   (em_ground
    :reader em_ground
    :initarg :em_ground
    :type cl:boolean
    :initform cl:nil)
   (em_open
    :reader em_open
    :initarg :em_open
    :type cl:boolean
    :initform cl:nil)
   (em_sky
    :reader em_sky
    :initarg :em_sky
    :type cl:boolean
    :initform cl:nil)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (agx
    :reader agx
    :initarg :agx
    :type cl:float
    :initform 0.0)
   (agy
    :reader agy
    :initarg :agy
    :type cl:float
    :initform 0.0)
   (agz
    :reader agz
    :initarg :agz
    :type cl:float
    :initform 0.0))
)

(cl:defclass FlightData (<FlightData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name flock_msgs-msg:<FlightData> is deprecated: use flock_msgs-msg:FlightData instead.")))

(cl:ensure-generic-function 'battery_percent-val :lambda-list '(m))
(cl:defmethod battery_percent-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:battery_percent-val is deprecated.  Use flock_msgs-msg:battery_percent instead.")
  (battery_percent m))

(cl:ensure-generic-function 'estimated_flight_time_remaining-val :lambda-list '(m))
(cl:defmethod estimated_flight_time_remaining-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:estimated_flight_time_remaining-val is deprecated.  Use flock_msgs-msg:estimated_flight_time_remaining instead.")
  (estimated_flight_time_remaining m))

(cl:ensure-generic-function 'flight_mode-val :lambda-list '(m))
(cl:defmethod flight_mode-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:flight_mode-val is deprecated.  Use flock_msgs-msg:flight_mode instead.")
  (flight_mode m))

(cl:ensure-generic-function 'flight_time-val :lambda-list '(m))
(cl:defmethod flight_time-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:flight_time-val is deprecated.  Use flock_msgs-msg:flight_time instead.")
  (flight_time m))

(cl:ensure-generic-function 'east_speed-val :lambda-list '(m))
(cl:defmethod east_speed-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:east_speed-val is deprecated.  Use flock_msgs-msg:east_speed instead.")
  (east_speed m))

(cl:ensure-generic-function 'north_speed-val :lambda-list '(m))
(cl:defmethod north_speed-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:north_speed-val is deprecated.  Use flock_msgs-msg:north_speed instead.")
  (north_speed m))

(cl:ensure-generic-function 'ground_speed-val :lambda-list '(m))
(cl:defmethod ground_speed-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:ground_speed-val is deprecated.  Use flock_msgs-msg:ground_speed instead.")
  (ground_speed m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:altitude-val is deprecated.  Use flock_msgs-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'equipment-val :lambda-list '(m))
(cl:defmethod equipment-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:equipment-val is deprecated.  Use flock_msgs-msg:equipment instead.")
  (equipment m))

(cl:ensure-generic-function 'high_temperature-val :lambda-list '(m))
(cl:defmethod high_temperature-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:high_temperature-val is deprecated.  Use flock_msgs-msg:high_temperature instead.")
  (high_temperature m))

(cl:ensure-generic-function 'em_ground-val :lambda-list '(m))
(cl:defmethod em_ground-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:em_ground-val is deprecated.  Use flock_msgs-msg:em_ground instead.")
  (em_ground m))

(cl:ensure-generic-function 'em_open-val :lambda-list '(m))
(cl:defmethod em_open-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:em_open-val is deprecated.  Use flock_msgs-msg:em_open instead.")
  (em_open m))

(cl:ensure-generic-function 'em_sky-val :lambda-list '(m))
(cl:defmethod em_sky-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:em_sky-val is deprecated.  Use flock_msgs-msg:em_sky instead.")
  (em_sky m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:pitch-val is deprecated.  Use flock_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:roll-val is deprecated.  Use flock_msgs-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:yaw-val is deprecated.  Use flock_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'agx-val :lambda-list '(m))
(cl:defmethod agx-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:agx-val is deprecated.  Use flock_msgs-msg:agx instead.")
  (agx m))

(cl:ensure-generic-function 'agy-val :lambda-list '(m))
(cl:defmethod agy-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:agy-val is deprecated.  Use flock_msgs-msg:agy instead.")
  (agy m))

(cl:ensure-generic-function 'agz-val :lambda-list '(m))
(cl:defmethod agz-val ((m <FlightData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader flock_msgs-msg:agz-val is deprecated.  Use flock_msgs-msg:agz instead.")
  (agz m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightData>)))
    "Constants for message type '<FlightData>"
  '((:FLIGHT_MODE_GROUND . 1)
    (:FLIGHT_MODE_HOVER . 6)
    (:FLIGHT_MODE_TAKING_OFF . 11)
    (:FLIGHT_MODE_LANDING . 12)
    (:FLIGHT_MODE_SPINNING_UP . 41)
    (:EQUIPMENT_OK . 0)
    (:EQUIPMENT_UNSTABLE . 21)
    (:EQUIPMENT_TIMER_EXCEEDED . 205))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightData)))
    "Constants for message type 'FlightData"
  '((:FLIGHT_MODE_GROUND . 1)
    (:FLIGHT_MODE_HOVER . 6)
    (:FLIGHT_MODE_TAKING_OFF . 11)
    (:FLIGHT_MODE_LANDING . 12)
    (:FLIGHT_MODE_SPINNING_UP . 41)
    (:EQUIPMENT_OK . 0)
    (:EQUIPMENT_UNSTABLE . 21)
    (:EQUIPMENT_TIMER_EXCEEDED . 205))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightData>) ostream)
  "Serializes a message object of type '<FlightData>"
  (cl:let* ((signed (cl:slot-value msg 'battery_percent)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'estimated_flight_time_remaining))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flight_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'flight_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'east_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'north_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ground_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'equipment)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'high_temperature) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'em_ground) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'em_open) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'em_sky) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'agz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightData>) istream)
  "Deserializes a message object of type '<FlightData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'battery_percent) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'estimated_flight_time_remaining) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flight_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'flight_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'east_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'north_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ground_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'equipment) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'high_temperature) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'em_ground) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'em_open) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'em_sky) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'agz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightData>)))
  "Returns string type for a message object of type '<FlightData>"
  "flock_msgs/FlightData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightData)))
  "Returns string type for a message object of type 'FlightData"
  "flock_msgs/FlightData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightData>)))
  "Returns md5sum for a message object of type '<FlightData>"
  "1fbfcd738c3afa96d840f05b5d17f7de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightData)))
  "Returns md5sum for a message object of type 'FlightData"
  "1fbfcd738c3afa96d840f05b5d17f7de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightData>)))
  "Returns full string definition for message of type '<FlightData>"
  (cl:format cl:nil "# Flight data -- experimental -- will change as we learn more~%# Gobot code seems to be the best reference~%~%# Battery state:~%int32 battery_percent                     # Remaining battery, 0-100~%float32 estimated_flight_time_remaining   # Remaining flight time, seconds~%~%# Flight modes:~%uint8 flight_mode_ground=1          # Motors off~%uint8 flight_mode_hover=6           # Hovering~%uint8 flight_mode_taking_off=11     # Taking off~%uint8 flight_mode_landing=12        # Landing~%uint8 flight_mode_spinning_up=41    # Spinning up the props, will take off soon~%uint8 flight_mode~%~%# Flight time:~%float32 flight_time                 # Flight time since power up, in seconds~%~%# Position and velocity, negative numbers mean \"no data\":~%float32 east_speed                  # meters/second~%float32 north_speed                 # meters/second~%float32 ground_speed                # meters/second~%float32 altitude                    # Height off the ground in meters~%~%# Equipment status:~%int32 equipment_ok=0                # Everything is OK~%int32 equipment_unstable=21         # The drone is unstable, tilted at an odd angle or upside down~%int32 equipment_timer_exceeded=205  # No input for 15 seconds, shutting down~%int32 equipment~%~%# Temperature:~%bool high_temperature               # It's getting warm in here~%~%# ???~%bool em_ground                      # ???~%bool em_open                        # ???~%bool em_sky                         # ???~%~%~%float32 pitch~%float32 roll~%float32 yaw~%float32 agx~%float32 agy~%float32 agz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightData)))
  "Returns full string definition for message of type 'FlightData"
  (cl:format cl:nil "# Flight data -- experimental -- will change as we learn more~%# Gobot code seems to be the best reference~%~%# Battery state:~%int32 battery_percent                     # Remaining battery, 0-100~%float32 estimated_flight_time_remaining   # Remaining flight time, seconds~%~%# Flight modes:~%uint8 flight_mode_ground=1          # Motors off~%uint8 flight_mode_hover=6           # Hovering~%uint8 flight_mode_taking_off=11     # Taking off~%uint8 flight_mode_landing=12        # Landing~%uint8 flight_mode_spinning_up=41    # Spinning up the props, will take off soon~%uint8 flight_mode~%~%# Flight time:~%float32 flight_time                 # Flight time since power up, in seconds~%~%# Position and velocity, negative numbers mean \"no data\":~%float32 east_speed                  # meters/second~%float32 north_speed                 # meters/second~%float32 ground_speed                # meters/second~%float32 altitude                    # Height off the ground in meters~%~%# Equipment status:~%int32 equipment_ok=0                # Everything is OK~%int32 equipment_unstable=21         # The drone is unstable, tilted at an odd angle or upside down~%int32 equipment_timer_exceeded=205  # No input for 15 seconds, shutting down~%int32 equipment~%~%# Temperature:~%bool high_temperature               # It's getting warm in here~%~%# ???~%bool em_ground                      # ???~%bool em_open                        # ???~%bool em_sky                         # ???~%~%~%float32 pitch~%float32 roll~%float32 yaw~%float32 agx~%float32 agy~%float32 agz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightData>))
  (cl:+ 0
     4
     4
     1
     4
     4
     4
     4
     4
     4
     1
     1
     1
     1
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightData>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightData
    (cl:cons ':battery_percent (battery_percent msg))
    (cl:cons ':estimated_flight_time_remaining (estimated_flight_time_remaining msg))
    (cl:cons ':flight_mode (flight_mode msg))
    (cl:cons ':flight_time (flight_time msg))
    (cl:cons ':east_speed (east_speed msg))
    (cl:cons ':north_speed (north_speed msg))
    (cl:cons ':ground_speed (ground_speed msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':equipment (equipment msg))
    (cl:cons ':high_temperature (high_temperature msg))
    (cl:cons ':em_ground (em_ground msg))
    (cl:cons ':em_open (em_open msg))
    (cl:cons ':em_sky (em_sky msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':agx (agx msg))
    (cl:cons ':agy (agy msg))
    (cl:cons ':agz (agz msg))
))
