;; Auto-generated. Do not edit!


(when (boundp 'flock_msgs::FlightData)
  (if (not (find-package "FLOCK_MSGS"))
    (make-package "FLOCK_MSGS"))
  (shadow 'FlightData (find-package "FLOCK_MSGS")))
(unless (find-package "FLOCK_MSGS::FLIGHTDATA")
  (make-package "FLOCK_MSGS::FLIGHTDATA"))

(in-package "ROS")
;;//! \htmlinclude FlightData.msg.html


(intern "*FLIGHT_MODE_GROUND*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*FLIGHT_MODE_GROUND* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*FLIGHT_MODE_GROUND* 1)
(intern "*FLIGHT_MODE_HOVER*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*FLIGHT_MODE_HOVER* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*FLIGHT_MODE_HOVER* 6)
(intern "*FLIGHT_MODE_TAKING_OFF*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*FLIGHT_MODE_TAKING_OFF* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*FLIGHT_MODE_TAKING_OFF* 11)
(intern "*FLIGHT_MODE_LANDING*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*FLIGHT_MODE_LANDING* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*FLIGHT_MODE_LANDING* 12)
(intern "*FLIGHT_MODE_SPINNING_UP*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*FLIGHT_MODE_SPINNING_UP* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*FLIGHT_MODE_SPINNING_UP* 41)
(intern "*EQUIPMENT_OK*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*EQUIPMENT_OK* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*EQUIPMENT_OK* 0)
(intern "*EQUIPMENT_UNSTABLE*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*EQUIPMENT_UNSTABLE* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*EQUIPMENT_UNSTABLE* 21)
(intern "*EQUIPMENT_TIMER_EXCEEDED*" (find-package "FLOCK_MSGS::FLIGHTDATA"))
(shadow '*EQUIPMENT_TIMER_EXCEEDED* (find-package "FLOCK_MSGS::FLIGHTDATA"))
(defconstant flock_msgs::FlightData::*EQUIPMENT_TIMER_EXCEEDED* 205)
(defclass flock_msgs::FlightData
  :super ros::object
  :slots (_battery_percent _estimated_flight_time_remaining _flight_mode _flight_time _east_speed _north_speed _ground_speed _altitude _equipment _high_temperature _em_ground _em_open _em_sky _pitch _roll _yaw _agx _agy _agz ))

(defmethod flock_msgs::FlightData
  (:init
   (&key
    ((:battery_percent __battery_percent) 0)
    ((:estimated_flight_time_remaining __estimated_flight_time_remaining) 0.0)
    ((:flight_mode __flight_mode) 0)
    ((:flight_time __flight_time) 0.0)
    ((:east_speed __east_speed) 0.0)
    ((:north_speed __north_speed) 0.0)
    ((:ground_speed __ground_speed) 0.0)
    ((:altitude __altitude) 0.0)
    ((:equipment __equipment) 0)
    ((:high_temperature __high_temperature) nil)
    ((:em_ground __em_ground) nil)
    ((:em_open __em_open) nil)
    ((:em_sky __em_sky) nil)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:yaw __yaw) 0.0)
    ((:agx __agx) 0.0)
    ((:agy __agy) 0.0)
    ((:agz __agz) 0.0)
    )
   (send-super :init)
   (setq _battery_percent (round __battery_percent))
   (setq _estimated_flight_time_remaining (float __estimated_flight_time_remaining))
   (setq _flight_mode (round __flight_mode))
   (setq _flight_time (float __flight_time))
   (setq _east_speed (float __east_speed))
   (setq _north_speed (float __north_speed))
   (setq _ground_speed (float __ground_speed))
   (setq _altitude (float __altitude))
   (setq _equipment (round __equipment))
   (setq _high_temperature __high_temperature)
   (setq _em_ground __em_ground)
   (setq _em_open __em_open)
   (setq _em_sky __em_sky)
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _yaw (float __yaw))
   (setq _agx (float __agx))
   (setq _agy (float __agy))
   (setq _agz (float __agz))
   self)
  (:battery_percent
   (&optional __battery_percent)
   (if __battery_percent (setq _battery_percent __battery_percent)) _battery_percent)
  (:estimated_flight_time_remaining
   (&optional __estimated_flight_time_remaining)
   (if __estimated_flight_time_remaining (setq _estimated_flight_time_remaining __estimated_flight_time_remaining)) _estimated_flight_time_remaining)
  (:flight_mode
   (&optional __flight_mode)
   (if __flight_mode (setq _flight_mode __flight_mode)) _flight_mode)
  (:flight_time
   (&optional __flight_time)
   (if __flight_time (setq _flight_time __flight_time)) _flight_time)
  (:east_speed
   (&optional __east_speed)
   (if __east_speed (setq _east_speed __east_speed)) _east_speed)
  (:north_speed
   (&optional __north_speed)
   (if __north_speed (setq _north_speed __north_speed)) _north_speed)
  (:ground_speed
   (&optional __ground_speed)
   (if __ground_speed (setq _ground_speed __ground_speed)) _ground_speed)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:equipment
   (&optional __equipment)
   (if __equipment (setq _equipment __equipment)) _equipment)
  (:high_temperature
   (&optional __high_temperature)
   (if __high_temperature (setq _high_temperature __high_temperature)) _high_temperature)
  (:em_ground
   (&optional __em_ground)
   (if __em_ground (setq _em_ground __em_ground)) _em_ground)
  (:em_open
   (&optional __em_open)
   (if __em_open (setq _em_open __em_open)) _em_open)
  (:em_sky
   (&optional __em_sky)
   (if __em_sky (setq _em_sky __em_sky)) _em_sky)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:agx
   (&optional __agx)
   (if __agx (setq _agx __agx)) _agx)
  (:agy
   (&optional __agy)
   (if __agy (setq _agy __agy)) _agy)
  (:agz
   (&optional __agz)
   (if __agz (setq _agz __agz)) _agz)
  (:serialization-length
   ()
   (+
    ;; int32 _battery_percent
    4
    ;; float32 _estimated_flight_time_remaining
    4
    ;; uint8 _flight_mode
    1
    ;; float32 _flight_time
    4
    ;; float32 _east_speed
    4
    ;; float32 _north_speed
    4
    ;; float32 _ground_speed
    4
    ;; float32 _altitude
    4
    ;; int32 _equipment
    4
    ;; bool _high_temperature
    1
    ;; bool _em_ground
    1
    ;; bool _em_open
    1
    ;; bool _em_sky
    1
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; float32 _yaw
    4
    ;; float32 _agx
    4
    ;; float32 _agy
    4
    ;; float32 _agz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _battery_percent
       (write-long _battery_percent s)
     ;; float32 _estimated_flight_time_remaining
       (sys::poke _estimated_flight_time_remaining (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _flight_mode
       (write-byte _flight_mode s)
     ;; float32 _flight_time
       (sys::poke _flight_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _east_speed
       (sys::poke _east_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _north_speed
       (sys::poke _north_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ground_speed
       (sys::poke _ground_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _equipment
       (write-long _equipment s)
     ;; bool _high_temperature
       (if _high_temperature (write-byte -1 s) (write-byte 0 s))
     ;; bool _em_ground
       (if _em_ground (write-byte -1 s) (write-byte 0 s))
     ;; bool _em_open
       (if _em_open (write-byte -1 s) (write-byte 0 s))
     ;; bool _em_sky
       (if _em_sky (write-byte -1 s) (write-byte 0 s))
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _agx
       (sys::poke _agx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _agy
       (sys::poke _agy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _agz
       (sys::poke _agz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _battery_percent
     (setq _battery_percent (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _estimated_flight_time_remaining
     (setq _estimated_flight_time_remaining (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _flight_mode
     (setq _flight_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _flight_time
     (setq _flight_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _east_speed
     (setq _east_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _north_speed
     (setq _north_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ground_speed
     (setq _ground_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _equipment
     (setq _equipment (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _high_temperature
     (setq _high_temperature (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _em_ground
     (setq _em_ground (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _em_open
     (setq _em_open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _em_sky
     (setq _em_sky (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _agx
     (setq _agx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _agy
     (setq _agy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _agz
     (setq _agz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get flock_msgs::FlightData :md5sum-) "1fbfcd738c3afa96d840f05b5d17f7de")
(setf (get flock_msgs::FlightData :datatype-) "flock_msgs/FlightData")
(setf (get flock_msgs::FlightData :definition-)
      "# Flight data -- experimental -- will change as we learn more
# Gobot code seems to be the best reference

# Battery state:
int32 battery_percent                     # Remaining battery, 0-100
float32 estimated_flight_time_remaining   # Remaining flight time, seconds

# Flight modes:
uint8 flight_mode_ground=1          # Motors off
uint8 flight_mode_hover=6           # Hovering
uint8 flight_mode_taking_off=11     # Taking off
uint8 flight_mode_landing=12        # Landing
uint8 flight_mode_spinning_up=41    # Spinning up the props, will take off soon
uint8 flight_mode

# Flight time:
float32 flight_time                 # Flight time since power up, in seconds

# Position and velocity, negative numbers mean \"no data\":
float32 east_speed                  # meters/second
float32 north_speed                 # meters/second
float32 ground_speed                # meters/second
float32 altitude                    # Height off the ground in meters

# Equipment status:
int32 equipment_ok=0                # Everything is OK
int32 equipment_unstable=21         # The drone is unstable, tilted at an odd angle or upside down
int32 equipment_timer_exceeded=205  # No input for 15 seconds, shutting down
int32 equipment

# Temperature:
bool high_temperature               # It's getting warm in here

# ???
bool em_ground                      # ???
bool em_open                        # ???
bool em_sky                         # ???


float32 pitch
float32 roll
float32 yaw
float32 agx
float32 agy
float32 agz
")



(provide :flock_msgs/FlightData "1fbfcd738c3afa96d840f05b5d17f7de")


