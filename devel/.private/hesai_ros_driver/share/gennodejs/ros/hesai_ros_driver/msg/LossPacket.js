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

class LossPacket {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.total_packet_count = null;
      this.total_packet_loss_count = null;
    }
    else {
      if (initObj.hasOwnProperty('total_packet_count')) {
        this.total_packet_count = initObj.total_packet_count
      }
      else {
        this.total_packet_count = 0;
      }
      if (initObj.hasOwnProperty('total_packet_loss_count')) {
        this.total_packet_loss_count = initObj.total_packet_loss_count
      }
      else {
        this.total_packet_loss_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LossPacket
    // Serialize message field [total_packet_count]
    bufferOffset = _serializer.uint32(obj.total_packet_count, buffer, bufferOffset);
    // Serialize message field [total_packet_loss_count]
    bufferOffset = _serializer.uint32(obj.total_packet_loss_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LossPacket
    let len;
    let data = new LossPacket(null);
    // Deserialize message field [total_packet_count]
    data.total_packet_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [total_packet_loss_count]
    data.total_packet_loss_count = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hesai_ros_driver/LossPacket';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '363355020f4e7cc5a0f379abeda225bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 total_packet_count
    uint32 total_packet_loss_count
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LossPacket(null);
    if (msg.total_packet_count !== undefined) {
      resolved.total_packet_count = msg.total_packet_count;
    }
    else {
      resolved.total_packet_count = 0
    }

    if (msg.total_packet_loss_count !== undefined) {
      resolved.total_packet_loss_count = msg.total_packet_loss_count;
    }
    else {
      resolved.total_packet_loss_count = 0
    }

    return resolved;
    }
};

module.exports = LossPacket;
