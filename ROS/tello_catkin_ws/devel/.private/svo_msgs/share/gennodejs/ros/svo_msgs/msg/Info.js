// Auto-generated. Do not edit!

// (in-package svo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.processing_time = null;
      this.keyframes = null;
      this.num_matches = null;
      this.tracking_quality = null;
      this.stage = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('processing_time')) {
        this.processing_time = initObj.processing_time
      }
      else {
        this.processing_time = 0.0;
      }
      if (initObj.hasOwnProperty('keyframes')) {
        this.keyframes = initObj.keyframes
      }
      else {
        this.keyframes = [];
      }
      if (initObj.hasOwnProperty('num_matches')) {
        this.num_matches = initObj.num_matches
      }
      else {
        this.num_matches = 0;
      }
      if (initObj.hasOwnProperty('tracking_quality')) {
        this.tracking_quality = initObj.tracking_quality
      }
      else {
        this.tracking_quality = 0;
      }
      if (initObj.hasOwnProperty('stage')) {
        this.stage = initObj.stage
      }
      else {
        this.stage = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [processing_time]
    bufferOffset = _serializer.float32(obj.processing_time, buffer, bufferOffset);
    // Serialize message field [keyframes]
    bufferOffset = _arraySerializer.int32(obj.keyframes, buffer, bufferOffset, null);
    // Serialize message field [num_matches]
    bufferOffset = _serializer.int32(obj.num_matches, buffer, bufferOffset);
    // Serialize message field [tracking_quality]
    bufferOffset = _serializer.int32(obj.tracking_quality, buffer, bufferOffset);
    // Serialize message field [stage]
    bufferOffset = _serializer.int32(obj.stage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Info
    let len;
    let data = new Info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [processing_time]
    data.processing_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [keyframes]
    data.keyframes = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [num_matches]
    data.num_matches = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tracking_quality]
    data.tracking_quality = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [stage]
    data.stage = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.keyframes.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'svo_msgs/Info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '175acf2e539a9219addbcbeafca8552f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header      header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.processing_time !== undefined) {
      resolved.processing_time = msg.processing_time;
    }
    else {
      resolved.processing_time = 0.0
    }

    if (msg.keyframes !== undefined) {
      resolved.keyframes = msg.keyframes;
    }
    else {
      resolved.keyframes = []
    }

    if (msg.num_matches !== undefined) {
      resolved.num_matches = msg.num_matches;
    }
    else {
      resolved.num_matches = 0
    }

    if (msg.tracking_quality !== undefined) {
      resolved.tracking_quality = msg.tracking_quality;
    }
    else {
      resolved.tracking_quality = 0
    }

    if (msg.stage !== undefined) {
      resolved.stage = msg.stage;
    }
    else {
      resolved.stage = 0
    }

    return resolved;
    }
};

module.exports = Info;
