# Flight data -- experimental -- will change as we learn more
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

# Position and velocity, negative numbers mean "no data":
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