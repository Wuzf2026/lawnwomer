// Auto-generated. Do not edit!

// (in-package hesai_ros_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UdpPacket = require('./UdpPacket.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UdpFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.packets = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('packets')) {
        this.packets = initObj.packets
      }
      else {
        this.packets = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UdpFrame
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [packets]
    // Serialize the length for message field [packets]
    bufferOffset = _serializer.uint32(obj.packets.length, buffer, bufferOffset);
    obj.packets.forEach((val) => {
      bufferOffset = UdpPacket.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UdpFrame
    let len;
    let data = new UdpFrame(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [packets]
    // Deserialize array length for message field [packets]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.packets = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.packets[i] = UdpPacket.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.packets.forEach((val) => {
      length += UdpPacket.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hesai_ros_driver/UdpFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '70c3ed4f4ae9144323298b04cc2c760b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    UdpPacket[] packets
    
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
    string frame_id
    
    ================================================================================
    MSG: hesai_ros_driver/UdpPacket
    # field		size(byte)
    # SOB 		2
    # angle		2
    # measure	5
    # block		SOB + angle + measure * 40
    # timestamp	4
    # factory	2
    # reserve	8
    # rpm		2
    # tail		timestamp + factory + reserve + rpm
    # packet	block * 6 + tail
    
    time stamp
    uint8[] data
    uint32 size
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UdpFrame(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.packets !== undefined) {
      resolved.packets = new Array(msg.packets.length);
      for (let i = 0; i < resolved.packets.length; ++i) {
        resolved.packets[i] = UdpPacket.Resolve(msg.packets[i]);
      }
    }
    else {
      resolved.packets = []
    }

    return resolved;
    }
};

module.exports = UdpFrame;
