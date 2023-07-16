// Auto-generated. Do not edit!

// (in-package drone_test.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class detection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.erro_x = null;
      this.erro_y = null;
      this.erro_z = null;
      this.flag = null;
      this.start_opencv = null;
    }
    else {
      if (initObj.hasOwnProperty('erro_x')) {
        this.erro_x = initObj.erro_x
      }
      else {
        this.erro_x = 0.0;
      }
      if (initObj.hasOwnProperty('erro_y')) {
        this.erro_y = initObj.erro_y
      }
      else {
        this.erro_y = 0.0;
      }
      if (initObj.hasOwnProperty('erro_z')) {
        this.erro_z = initObj.erro_z
      }
      else {
        this.erro_z = 0.0;
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = 0;
      }
      if (initObj.hasOwnProperty('start_opencv')) {
        this.start_opencv = initObj.start_opencv
      }
      else {
        this.start_opencv = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type detection
    // Serialize message field [erro_x]
    bufferOffset = _serializer.float64(obj.erro_x, buffer, bufferOffset);
    // Serialize message field [erro_y]
    bufferOffset = _serializer.float64(obj.erro_y, buffer, bufferOffset);
    // Serialize message field [erro_z]
    bufferOffset = _serializer.float64(obj.erro_z, buffer, bufferOffset);
    // Serialize message field [flag]
    bufferOffset = _serializer.uint8(obj.flag, buffer, bufferOffset);
    // Serialize message field [start_opencv]
    bufferOffset = _serializer.uint8(obj.start_opencv, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type detection
    let len;
    let data = new detection(null);
    // Deserialize message field [erro_x]
    data.erro_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [erro_y]
    data.erro_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [erro_z]
    data.erro_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [flag]
    data.flag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [start_opencv]
    data.start_opencv = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_test/detection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '156cdb81e2b7ac989e48faaa7c1e6124';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 erro_x
    float64 erro_y
    float64 erro_z
    uint8 flag
    uint8 start_opencv
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new detection(null);
    if (msg.erro_x !== undefined) {
      resolved.erro_x = msg.erro_x;
    }
    else {
      resolved.erro_x = 0.0
    }

    if (msg.erro_y !== undefined) {
      resolved.erro_y = msg.erro_y;
    }
    else {
      resolved.erro_y = 0.0
    }

    if (msg.erro_z !== undefined) {
      resolved.erro_z = msg.erro_z;
    }
    else {
      resolved.erro_z = 0.0
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = 0
    }

    if (msg.start_opencv !== undefined) {
      resolved.start_opencv = msg.start_opencv;
    }
    else {
      resolved.start_opencv = 0
    }

    return resolved;
    }
};

module.exports = detection;
