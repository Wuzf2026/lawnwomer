// Auto-generated. Do not edit!

// (in-package hesai_ros_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Firetime {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Array(512).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Firetime
    // Check that the constant length array field [data] has the right length
    if (obj.data.length !== 512) {
      throw new Error('Unable to serialize array field data - length must be 512')
    }
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float64(obj.data, buffer, bufferOffset, 512);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Firetime
    let len;
    let data = new Firetime(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float64(buffer, bufferOffset, 512)
    return data;
  }

  static getMessageSize(object) {
    return 4096;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hesai_ros_driver/Firetime';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '869425aed297bb57ab8d72e8d4c6bc7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[512] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Firetime(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = new Array(512).fill(0)
    }

    return resolved;
    }
};

module.exports = Firetime;
