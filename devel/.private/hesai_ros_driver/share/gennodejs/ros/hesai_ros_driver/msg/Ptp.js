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

class Ptp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ptp_lock_offset = null;
      this.ptp_status = null;
    }
    else {
      if (initObj.hasOwnProperty('ptp_lock_offset')) {
        this.ptp_lock_offset = initObj.ptp_lock_offset
      }
      else {
        this.ptp_lock_offset = 0;
      }
      if (initObj.hasOwnProperty('ptp_status')) {
        this.ptp_status = initObj.ptp_status
      }
      else {
        this.ptp_status = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ptp
    // Serialize message field [ptp_lock_offset]
    bufferOffset = _serializer.uint8(obj.ptp_lock_offset, buffer, bufferOffset);
    // Check that the constant length array field [ptp_status] has the right length
    if (obj.ptp_status.length !== 16) {
      throw new Error('Unable to serialize array field ptp_status - length must be 16')
    }
    // Serialize message field [ptp_status]
    bufferOffset = _arraySerializer.uint8(obj.ptp_status, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ptp
    let len;
    let data = new Ptp(null);
    // Deserialize message field [ptp_lock_offset]
    data.ptp_lock_offset = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ptp_status]
    data.ptp_status = _arrayDeserializer.uint8(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hesai_ros_driver/Ptp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17a101cc667a4bf8eccaf77e6e093ba0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 ptp_lock_offset
    uint8[16] ptp_status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ptp(null);
    if (msg.ptp_lock_offset !== undefined) {
      resolved.ptp_lock_offset = msg.ptp_lock_offset;
    }
    else {
      resolved.ptp_lock_offset = 0
    }

    if (msg.ptp_status !== undefined) {
      resolved.ptp_status = msg.ptp_status;
    }
    else {
      resolved.ptp_status = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = Ptp;
