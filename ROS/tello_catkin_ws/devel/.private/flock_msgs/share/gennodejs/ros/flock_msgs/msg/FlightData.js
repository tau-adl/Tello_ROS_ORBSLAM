// Auto-generated. Do not edit!

// (in-package flock_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FlightData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.battery_percent = null;
      this.estimated_flight_time_remaining = null;
      this.flight_mode = null;
      this.flight_time = null;
      this.east_speed = null;
      this.north_speed = null;
      this.ground_speed = null;
      this.altitude = null;
      this.equipment = null;
      this.high_temperature = null;
      this.em_ground = null;
      this.em_open = null;
      this.em_sky = null;
      this.pitch = null;
      this.roll = null;
      this.yaw = null;
      this.agx = null;
      this.agy = null;
      this.agz = null;
    }
    else {
      if (initObj.hasOwnProperty('battery_percent')) {
        this.battery_percent = initObj.battery_percent
      }
      else {
        this.battery_percent = 0;
      }
      if (initObj.hasOwnProperty('estimated_flight_time_remaining')) {
        this.estimated_flight_time_remaining = initObj.estimated_flight_time_remaining
      }
      else {
        this.estimated_flight_time_remaining = 0.0;
      }
      if (initObj.hasOwnProperty('flight_mode')) {
        this.flight_mode = initObj.flight_mode
      }
      else {
        this.flight_mode = 0;
      }
      if (initObj.hasOwnProperty('flight_time')) {
        this.flight_time = initObj.flight_time
      }
      else {
        this.flight_time = 0.0;
      }
      if (initObj.hasOwnProperty('east_speed')) {
        this.east_speed = initObj.east_speed
      }
      else {
        this.east_speed = 0.0;
      }
      if (initObj.hasOwnProperty('north_speed')) {
        this.north_speed = initObj.north_speed
      }
      else {
        this.north_speed = 0.0;
      }
      if (initObj.hasOwnProperty('ground_speed')) {
        this.ground_speed = initObj.ground_speed
      }
      else {
        this.ground_speed = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('equipment')) {
        this.equipment = initObj.equipment
      }
      else {
        this.equipment = 0;
      }
      if (initObj.hasOwnProperty('high_temperature')) {
        this.high_temperature = initObj.high_temperature
      }
      else {
        this.high_temperature = false;
      }
      if (initObj.hasOwnProperty('em_ground')) {
        this.em_ground = initObj.em_ground
      }
      else {
        this.em_ground = false;
      }
      if (initObj.hasOwnProperty('em_open')) {
        this.em_open = initObj.em_open
      }
      else {
        this.em_open = false;
      }
      if (initObj.hasOwnProperty('em_sky')) {
        this.em_sky = initObj.em_sky
      }
      else {
        this.em_sky = false;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('agx')) {
        this.agx = initObj.agx
      }
      else {
        this.agx = 0.0;
      }
      if (initObj.hasOwnProperty('agy')) {
        this.agy = initObj.agy
      }
      else {
        this.agy = 0.0;
      }
      if (initObj.hasOwnProperty('agz')) {
        this.agz = initObj.agz
      }
      else {
        this.agz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlightData
    // Serialize message field [battery_percent]
    bufferOffset = _serializer.int32(obj.battery_percent, buffer, bufferOffset);
    // Serialize message field [estimated_flight_time_remaining]
    bufferOffset = _serializer.float32(obj.estimated_flight_time_remaining, buffer, bufferOffset);
    // Serialize message field [flight_mode]
    bufferOffset = _serializer.uint8(obj.flight_mode, buffer, bufferOffset);
    // Serialize message field [flight_time]
    bufferOffset = _serializer.float32(obj.flight_time, buffer, bufferOffset);
    // Serialize message field [east_speed]
    bufferOffset = _serializer.float32(obj.east_speed, buffer, bufferOffset);
    // Serialize message field [north_speed]
    bufferOffset = _serializer.float32(obj.north_speed, buffer, bufferOffset);
    // Serialize message field [ground_speed]
    bufferOffset = _serializer.float32(obj.ground_speed, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float32(obj.altitude, buffer, bufferOffset);
    // Serialize message field [equipment]
    bufferOffset = _serializer.int32(obj.equipment, buffer, bufferOffset);
    // Serialize message field [high_temperature]
    bufferOffset = _serializer.bool(obj.high_temperature, buffer, bufferOffset);
    // Serialize message field [em_ground]
    bufferOffset = _serializer.bool(obj.em_ground, buffer, bufferOffset);
    // Serialize message field [em_open]
    bufferOffset = _serializer.bool(obj.em_open, buffer, bufferOffset);
    // Serialize message field [em_sky]
    bufferOffset = _serializer.bool(obj.em_sky, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [agx]
    bufferOffset = _serializer.float32(obj.agx, buffer, bufferOffset);
    // Serialize message field [agy]
    bufferOffset = _serializer.float32(obj.agy, buffer, bufferOffset);
    // Serialize message field [agz]
    bufferOffset = _serializer.float32(obj.agz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlightData
    let len;
    let data = new FlightData(null);
    // Deserialize message field [battery_percent]
    data.battery_percent = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [estimated_flight_time_remaining]
    data.estimated_flight_time_remaining = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [flight_mode]
    data.flight_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flight_time]
    data.flight_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [east_speed]
    data.east_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [north_speed]
    data.north_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ground_speed]
    data.ground_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [equipment]
    data.equipment = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [high_temperature]
    data.high_temperature = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [em_ground]
    data.em_ground = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [em_open]
    data.em_open = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [em_sky]
    data.em_sky = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [agx]
    data.agx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [agy]
    data.agy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [agz]
    data.agz = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'flock_msgs/FlightData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1fbfcd738c3afa96d840f05b5d17f7de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlightData(null);
    if (msg.battery_percent !== undefined) {
      resolved.battery_percent = msg.battery_percent;
    }
    else {
      resolved.battery_percent = 0
    }

    if (msg.estimated_flight_time_remaining !== undefined) {
      resolved.estimated_flight_time_remaining = msg.estimated_flight_time_remaining;
    }
    else {
      resolved.estimated_flight_time_remaining = 0.0
    }

    if (msg.flight_mode !== undefined) {
      resolved.flight_mode = msg.flight_mode;
    }
    else {
      resolved.flight_mode = 0
    }

    if (msg.flight_time !== undefined) {
      resolved.flight_time = msg.flight_time;
    }
    else {
      resolved.flight_time = 0.0
    }

    if (msg.east_speed !== undefined) {
      resolved.east_speed = msg.east_speed;
    }
    else {
      resolved.east_speed = 0.0
    }

    if (msg.north_speed !== undefined) {
      resolved.north_speed = msg.north_speed;
    }
    else {
      resolved.north_speed = 0.0
    }

    if (msg.ground_speed !== undefined) {
      resolved.ground_speed = msg.ground_speed;
    }
    else {
      resolved.ground_speed = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.equipment !== undefined) {
      resolved.equipment = msg.equipment;
    }
    else {
      resolved.equipment = 0
    }

    if (msg.high_temperature !== undefined) {
      resolved.high_temperature = msg.high_temperature;
    }
    else {
      resolved.high_temperature = false
    }

    if (msg.em_ground !== undefined) {
      resolved.em_ground = msg.em_ground;
    }
    else {
      resolved.em_ground = false
    }

    if (msg.em_open !== undefined) {
      resolved.em_open = msg.em_open;
    }
    else {
      resolved.em_open = false
    }

    if (msg.em_sky !== undefined) {
      resolved.em_sky = msg.em_sky;
    }
    else {
      resolved.em_sky = false
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.agx !== undefined) {
      resolved.agx = msg.agx;
    }
    else {
      resolved.agx = 0.0
    }

    if (msg.agy !== undefined) {
      resolved.agy = msg.agy;
    }
    else {
      resolved.agy = 0.0
    }

    if (msg.agz !== undefined) {
      resolved.agz = msg.agz;
    }
    else {
      resolved.agz = 0.0
    }

    return resolved;
    }
};

// Constants for message
FlightData.Constants = {
  FLIGHT_MODE_GROUND: 1,
  FLIGHT_MODE_HOVER: 6,
  FLIGHT_MODE_TAKING_OFF: 11,
  FLIGHT_MODE_LANDING: 12,
  FLIGHT_MODE_SPINNING_UP: 41,
  EQUIPMENT_OK: 0,
  EQUIPMENT_UNSTABLE: 21,
  EQUIPMENT_TIMER_EXCEEDED: 205,
}

module.exports = FlightData;
