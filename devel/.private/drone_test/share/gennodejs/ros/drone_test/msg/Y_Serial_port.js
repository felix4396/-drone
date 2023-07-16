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

class Y_Serial_port {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.tar1 = null;
      this.tar2 = null;
      this.if_takeoff = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('tar1')) {
        this.tar1 = initObj.tar1
      }
      else {
        this.tar1 = 0;
      }
      if (initObj.hasOwnProperty('tar2')) {
        this.tar2 = initObj.tar2
      }
      else {
        this.tar2 = 0;
      }
      if (initObj.hasOwnProperty('if_takeoff')) {
        this.if_takeoff = initObj.if_takeoff
      }
      else {
        this.if_takeoff = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Y_Serial_port
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Serialize message field [tar1]
    bufferOffset = _serializer.int8(obj.tar1, buffer, bufferOffset);
    // Serialize message field [tar2]
    bufferOffset = _serializer.int8(obj.tar2, buffer, bufferOffset);
    // Serialize message field [if_takeoff]
    bufferOffset = _serializer.int8(obj.if_takeoff, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Y_Serial_port
    let len;
    let data = new Y_Serial_port(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [tar1]
    data.tar1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [tar2]
    data.tar2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [if_takeoff]
    data.if_takeoff = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_test/Y_Serial_port';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae4cda509b27984341a3ec103b7367d7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 mode
    int8 tar1
    int8 tar2
    int8 if_takeoff 
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Y_Serial_port(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.tar1 !== undefined) {
      resolved.tar1 = msg.tar1;
    }
    else {
      resolved.tar1 = 0
    }

    if (msg.tar2 !== undefined) {
      resolved.tar2 = msg.tar2;
    }
    else {
      resolved.tar2 = 0
    }

    if (msg.if_takeoff !== undefined) {
      resolved.if_takeoff = msg.if_takeoff;
    }
    else {
      resolved.if_takeoff = 0
    }

    return resolved;
    }
};

module.exports = Y_Serial_port;
