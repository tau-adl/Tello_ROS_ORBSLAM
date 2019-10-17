// Auto-generated. Do not edit!

// (in-package svo_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Feature {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u = null;
      this.v = null;
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0.0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Feature
    // Serialize message field [u]
    bufferOffset = _serializer.float32(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.float32(obj.v, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Feature
    let len;
    let data = new Feature(null);
    // Deserialize message field [u]
    data.u = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'svo_msgs/Feature';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f7761c191bddfddcd2f99bea993a700';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 u
    float32 v
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Feature(null);
    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0.0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

module.exports = Feature;
